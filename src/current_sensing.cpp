#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_err.h"

/*
  ============================================================================
  INTEGRATED BUCK CHARGER & CURRENT MONITOR
  Pins:
    - PWM High Side: GPIO 19
    - PWM Low Side:  GPIO 18
    - Solar Current: GPIO 13 (ADC2_CH4) -> Note WiFi warning in code comments!
    - Bat Current:   GPIO 12 (ADC2_CH5) -> Note WiFi warning in code comments!
  ============================================================================
*/

// ================== PINS ==================
#define PWM_H_PIN     19
#define PWM_L_PIN     18
#define SOLAR_I_PIN   13   // ADC2_CH4 -- WiFi conflict warning
#define BAT_I_PIN     12   // ADC2_CH5 -- WiFi conflict warning

// ================== PWM SETTINGS ==================
#define LEDC_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_SEL        LEDC_TIMER_0
#define LEDC_CH_H             LEDC_CHANNEL_0
#define LEDC_CH_L             LEDC_CHANNEL_1
#define LEDC_RESOLUTION_BITS  LEDC_TIMER_10_BIT

static const uint32_t PWM_PERIOD_COUNTS = 1 << 10;
static float PWM_FREQ_HZ  = 30000.0f;   // 30 kHz
static float DEAD_TIME_US = 2.0f;
static int currentDuty    = 60;         // Safe starting duty for 25V -> 12V

// ================== ACS758 SENSING CONFIG ==================
const float VCC_SENSOR = 3.30f;   // Measured VCC feeding U7/U8
const float SENSITIVITY_V_PER_A = 0.020f * (VCC_SENSOR / 5.0f);   // ~0.0132 V/A starting estimate
float QUIESCENT_VOLTAGE = 1.56 ;                      // ~1.65V starting estimate

const float VREF    = 3.30f;   // Measure actual 3.3V rail for accuracy
const int   SAMPLES = 32;      // Block-average samples per reading

// ================== FILTERING VARIABLES ==================
float solarCurrentFiltered = 0.0f;
float batCurrentFiltered   = 0.0f;
const float EMA_ALPHA = 0.2f;

// ================== NON-BLOCKING TIMERS ==================
unsigned long lastCurrentReadTime = 0;
const unsigned long CURRENT_READ_INTERVAL = 200; // Read current every 200ms

// ================== FUNCTION DECLARATIONS ==================
void checkErr(const char* what, esp_err_t err);
void updatePWM(int duty);
void printStatus();
float readAveragedAdcVoltage(int pin);
float voltageToCurrent(float v_adc);
void calibrateZero();
void handleSerialCommands();

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== Integrated 25V -> 12V Buck Converter & Current Monitor ===");
    Serial.println(">>> Start low and increase duty slowly while watching current <<<\n");

    // Initialize ADC
    analogReadResolution(12);
    analogSetPinAttenuation(SOLAR_I_PIN, ADC_11db);
    analogSetPinAttenuation(BAT_I_PIN,   ADC_11db);

    // Initialize PWM Timer
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = LEDC_MODE;
    timer_conf.duty_resolution = LEDC_RESOLUTION_BITS;
    timer_conf.timer_num       = LEDC_TIMER_SEL;
    timer_conf.freq_hz         = (uint32_t)PWM_FREQ_HZ;
    timer_conf.clk_cfg         = LEDC_USE_APB_CLK;
    checkErr("timer_config", ledc_timer_config(&timer_conf));

    // High side channel
    ledc_channel_config_t ch_h = {};
    ch_h.gpio_num   = PWM_H_PIN;
    ch_h.speed_mode = LEDC_MODE;
    ch_h.channel    = LEDC_CH_H;
    ch_h.timer_sel  = LEDC_TIMER_SEL;
    checkErr("ch_h", ledc_channel_config(&ch_h));

    // Low side channel
    ledc_channel_config_t ch_l = ch_h;
    ch_l.gpio_num = PWM_L_PIN;
    ch_l.channel  = LEDC_CH_L;
    checkErr("ch_l", ledc_channel_config(&ch_l));

    // Apply startup PWM duty cycle
    updatePWM(currentDuty);
    printStatus();
}

void loop() {
    // 1. Listen for user input commands
    handleSerialCommands();

    // 2. Sample and display current metrics using a non-blocking interval
    if (millis() - lastCurrentReadTime >= CURRENT_READ_INTERVAL) {
        lastCurrentReadTime = millis();

        float v_adc_solar = readAveragedAdcVoltage(SOLAR_I_PIN);
        float v_adc_bat   = readAveragedAdcVoltage(BAT_I_PIN);

        float solarCurrentRaw = voltageToCurrent(v_adc_solar);
        float batCurrentRaw   = voltageToCurrent(v_adc_bat);

        solarCurrentFiltered = (EMA_ALPHA * solarCurrentRaw) + ((1.0f - EMA_ALPHA) * solarCurrentFiltered);
        batCurrentFiltered   = (EMA_ALPHA * batCurrentRaw)   + ((1.0f - EMA_ALPHA) * batCurrentFiltered);

        Serial.printf("Solar VIOUT: %.3f V  ->  I: %.2f A (raw)  %.2f A (filt) | ", v_adc_solar, solarCurrentRaw, solarCurrentFiltered);
        Serial.printf("Bat VIOUT: %.3f V  ->  I: %.2f A (raw)  %.2f A (filt) | Duty: %d%%\n", v_adc_bat, batCurrentRaw, batCurrentFiltered, currentDuty);
    }
}

// ================== PWM HELPERS ==================
void updatePWM(int duty) {
    uint32_t T = PWM_PERIOD_COUNTS;
    float periodUs = 1000000.0f / PWM_FREQ_HZ;
    uint32_t DT = (uint32_t)((DEAD_TIME_US / periodUs) * (float)T + 0.5f);

    uint32_t D = (uint32_t)((duty / 100.0f) * (float)T);
    if (D + 2 * DT >= T) D = T - 2 * DT - 1;
    if (D > T) D = T;

    ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CH_H, D, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CH_H);

    uint32_t lowStart = D + DT;
    uint32_t lowDuty  = T - DT - lowStart;

    ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CH_L, lowDuty, lowStart);
    ledc_update_duty(LEDC_MODE, LEDC_CH_L);

    currentDuty = duty;
    Serial.printf(">> Duty successfully changed to: %d%%\n", duty);
}

void printStatus() {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.printf("PWM Frequency : 30 kHz\n");
    Serial.printf("Current Duty  : %d%%\n", currentDuty);
    Serial.printf("Quiescent V   : %.4f V\n", QUIESCENT_VOLTAGE);
    Serial.println("Commands:");
    Serial.println("  d XX   -> Set duty cycle percent (e.g., d 45)");
    Serial.println("  z      -> Zero-calibrate current sensors (Ensure NO load/source connected!)");
    Serial.println("  s      -> Reprint this status table");
    Serial.println("=====================\n");
}

void checkErr(const char* what, esp_err_t err) {
    if (err != ESP_OK) {
        Serial.printf("ERROR in %s: %s\n", what, esp_err_to_name(err));
        while (true) { delay(1000); }
    }
}

// ================== CURRENT SENSING HELPERS ==================
float readAveragedAdcVoltage(int pin) {
    long sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        sum += analogRead(pin);
        delayMicroseconds(100);
    }
    float raw = (float)sum / SAMPLES;
    return raw * (VREF / 4095.0f);
}

float voltageToCurrent(float v_adc) {
    return (v_adc - QUIESCENT_VOLTAGE) / SENSITIVITY_V_PER_A;
}

void calibrateZero() {
    Serial.println("\n[Calibrating zero-current offset... ensure NO current is flowing]");
    delay(200);
    float v_solar = readAveragedAdcVoltage(SOLAR_I_PIN);
    float v_bat   = readAveragedAdcVoltage(BAT_I_PIN);
    
    Serial.printf("Measured quiescent -> Solar: %.4f V | Bat: %.4f V\n", v_solar, v_bat);
    QUIESCENT_VOLTAGE = (v_solar + v_bat) / 2.0f;
    Serial.printf("QUIESCENT_VOLTAGE updated to %.4f V\n\n", QUIESCENT_VOLTAGE);
}

// ================== SERIAL COMMAND PARSER ==================
void handleSerialCommands() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("z")) {
            calibrateZero();
        } 
        else if (input.equalsIgnoreCase("s")) {
            printStatus();
        } 
        else if (input.startsWith("d ") || input.startsWith("D ")) {
            int newDuty = input.substring(2).toInt();
            // Bound safety check (5% to 90%)
            if (newDuty >= 5 && newDuty <= 90) {
                updatePWM(newDuty);
            } else {
                Serial.println("Error: Duty must be between 5% and 90%");
            }
        }
    }
}
