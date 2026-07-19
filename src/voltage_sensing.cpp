/*
  ============================================================================
  ESP32 BUCK CHARGER - MANUAL DUTY CONTROL + VOLTAGE SENSING
  Input: 25V Power Supply -> 12V Battery
  30 kHz with live Serial duty adjustment
  PV/Battery voltage sensing via AMC1311 (VOUTP-only, VOUTN left floating)
  ============================================================================
  VOLTAGE SENSE NOTES:
    - VOUTP connects to GPIO34 (solar) / GPIO35 (battery), VOUTN floats.
    - VIN = (VOUTP - VCM) / 0.5, where VCM is THIS chip's real quiescent
      output (NOT the datasheet's 1.44V typical -- verified to vary chip to
      chip, e.g. ~1.42V measured on this board).
    - Defaults below (VCM_SOLAR/VCM_BAT = 1.42) are starting estimates from
      prior bench measurement. For best accuracy, re-calibrate on your board:
      ground both IN pins (zero input), then send 'z' to capture the real
      VOUTP quiescent value per channel.
    - ALPHA_PV/ALPHA_BAT must match your actual resistor divider ratios.
  ============================================================================
*/

#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_err.h"

// ================== PWM PINS ==================
#define PWM_H_PIN   19
#define PWM_L_PIN   18

// ================== VOLTAGE SENSE PINS (AMC1311, ADC1) ==================
#define SOLAR_V_PIN 34   // ADC1_CH6 <- AMC1311 VOUTP (solar)
#define BAT_V_PIN   35   // ADC1_CH7 <- AMC1311 VOUTP (battery)

// LEDC Settings
#define LEDC_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_SEL        LEDC_TIMER_0
#define LEDC_CH_H             LEDC_CHANNEL_0
#define LEDC_CH_L             LEDC_CHANNEL_1
#define LEDC_RESOLUTION_BITS  LEDC_TIMER_10_BIT

static const uint32_t PWM_PERIOD_COUNTS = 1 << 10;

// PWM Parameters
static float PWM_FREQ_HZ  = 30000.0f;   // 30 kHz
static float DEAD_TIME_US = 2.0f;
static int currentDuty    = 40;         // Safe starting duty for 25V -> 12V

// ================== AMC1311 VOLTAGE SENSE CONFIG ==================
const float GAIN_FACTOR = 0.5f;    // VOUTP-only readout (half of full differential gain)
float VCM_SOLAR = 1.42f;           // Chip-specific quiescent VOUTP -- calibrate with 'z'
float VCM_BAT   = 1.42f;           // Chip-specific quiescent VOUTP -- calibrate with 'z'

const float ALPHA_PV  = 1.0f / 31.0f;   // PV divider ratio -- confirm against your board
const float ALPHA_BAT = 1.0f / 31.0f;   // BAT divider ratio -- confirm against your board
const float SCALE_PV  = 1.0f / (ALPHA_PV );
const float SCALE_BAT = 1.0f / (ALPHA_BAT);

const float VREF      = 3.30f;   // Measure actual 3.3V rail for accuracy
const int   V_SAMPLES = 16;      // Voltage channel averaging

unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 500;   // Voltage print every 500ms

void checkErr(const char* what, esp_err_t err) {
  if (err != ESP_OK) {
    Serial.print("ERROR in ");
    Serial.print(what);
    Serial.print(": ");
    Serial.println(esp_err_to_name(err));
    while (true) { delay(1000); }
  }
}

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
  Serial.print("Duty changed to: ");
  Serial.print(duty);
  Serial.println("%");
}

// ================== VOLTAGE SENSE FUNCTIONS ==================
float readAveragedAdc(int pin) {
  long sum = 0;
  for (int i = 0; i < V_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(150);
  }
  float raw = (float)sum / V_SAMPLES;
  return raw * (VREF / 4095.0f);
}

float readSolarVoltage() {
  float v_adc = readAveragedAdc(SOLAR_V_PIN);
  float v_in  = (v_adc - VCM_SOLAR) / GAIN_FACTOR;
  if (v_in < 0) v_in = 0;
  return v_in * SCALE_PV;
}

float readBatVoltage() {
  float v_adc = readAveragedAdc(BAT_V_PIN) + 0.1;
  float v_in  = (v_adc - VCM_BAT) / GAIN_FACTOR;
  if (v_in < 0) v_in = 0;
  return v_in * SCALE_BAT;
}

void calibrateVCM() {
  Serial.println("\n[Calibrating VCM... ensure BOTH AMC1311 IN pins are grounded (zero input)]");
  delay(200);
  float vSolar = readAveragedAdc(SOLAR_V_PIN);
  float vBat   = readAveragedAdc(BAT_V_PIN);
  VCM_SOLAR = vSolar;
  VCM_BAT   = vBat;
  Serial.print("VCM_SOLAR updated to: "); Serial.println(vSolar, 4);
  Serial.print("VCM_BAT updated to:   "); Serial.println(vBat, 4);
  Serial.println("Calibration stored.\n");
}

void printStatus() {
  Serial.println("\n=== STATUS ===");
  Serial.print("Frequency : 30 kHz");
  Serial.print("\nDuty      : ");
  Serial.print(currentDuty);
  Serial.println("%");
  Serial.print("VCM_SOLAR : "); Serial.println(VCM_SOLAR, 4);
  Serial.print("VCM_BAT   : "); Serial.println(VCM_BAT, 4);
  Serial.println("Commands:");
  Serial.println("  d XX   -> set duty (e.g. d 48)");
  Serial.println("  z      -> calibrate VCM (ground both IN pins first)");
  Serial.println("  s      -> show status");
  Serial.println("==============\n");
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n=== 25V -> 12V Battery Charger (Manual Control + Voltage Sense) ===");
  Serial.println(">>> Start low and increase duty slowly while watching voltage <<<\n");

  // ADC setup (Arduino-native API -- avoids legacy/new driver conflict)
  analogReadResolution(12);
  analogSetPinAttenuation(SOLAR_V_PIN, ADC_11db);
  analogSetPinAttenuation(BAT_V_PIN,   ADC_11db);

  // Timer setup
  ledc_timer_config_t timer_conf = {};
  timer_conf.speed_mode      = LEDC_MODE;
  timer_conf.duty_resolution = LEDC_RESOLUTION_BITS;
  timer_conf.timer_num       = LEDC_TIMER_SEL;
  timer_conf.freq_hz         = (uint32_t)PWM_FREQ_HZ;
  timer_conf.clk_cfg         = LEDC_USE_APB_CLK;
  checkErr("timer_config", ledc_timer_config(&timer_conf));

  // High side
  ledc_channel_config_t ch_h = {};
  ch_h.gpio_num   = PWM_H_PIN;
  ch_h.speed_mode = LEDC_MODE;
  ch_h.channel    = LEDC_CH_H;
  ch_h.timer_sel  = LEDC_TIMER_SEL;
  checkErr("ch_h", ledc_channel_config(&ch_h));

  // Low side
  ledc_channel_config_t ch_l = ch_h;
  ch_l.gpio_num = PWM_L_PIN;
  ch_l.channel  = LEDC_CH_L;
  checkErr("ch_l", ledc_channel_config(&ch_l));

  updatePWM(currentDuty);
  printStatus();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("d ") || input.startsWith("D ")) {
      int newDuty = input.substring(2).toInt();
      if (newDuty >= 5 && newDuty <= 70) {
        updatePWM(newDuty);
      } else {
        Serial.println("Duty must be between 5 and 70");
      }
    } 
    else if (input.equalsIgnoreCase("s")) {
      printStatus();
    }
    else if (input.equalsIgnoreCase("z")) {
      calibrateVCM();
    }
  }

  // Voltage telemetry (non-blocking)
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = millis();
    float pvVoltage  = readSolarVoltage();
    float batVoltage = readBatVoltage();
    Serial.print("[Duty: ");
    Serial.print(currentDuty);
    Serial.print("%] PV In: ");
    Serial.print(pvVoltage, 2);
    Serial.print(" V | BAT Out: ");
    Serial.print(batVoltage, 2);
    Serial.println(" V");
  }
}
