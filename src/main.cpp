#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_err.h"

/*
  ============================================================================
  ESP32 MPPT SOLAR CHARGE CONTROLLER - Perturb & Observe
  Built from your latest calibrated voltage-sense + current-sense code.
  ============================================================================
  PINS:
    PWM High Side   : GPIO 19
    PWM Low Side    : GPIO 18
    Solar Voltage   : GPIO 34 (ADC1_CH6) <- AMC1311 VOUTP (single-ended)
    Bat   Voltage   : GPIO 35 (ADC1_CH7) <- AMC1311 VOUTP (single-ended)
    Solar Current   : GPIO 13 (ADC2_CH4) <- ACS758 VIOUT  [WiFi conflict!]
    Bat   Current   : GPIO 12 (ADC2_CH5) <- ACS758 VIOUT  [WiFi conflict!]

  CARRIED OVER FROM YOUR LATEST CALIBRATION:
    - Voltage: SCALE_PV/SCALE_BAT = 1/ALPHA (no extra GAIN_FACTOR folded in --
      GAIN_FACTOR is already applied when recovering VIN from VOUTP).
    - Voltage: readBatVoltage() adds a +0.1V correction to the raw ADC read
      before the VCM subtraction (kept exactly as you tuned it). Solar
      channel does NOT get this offset -- also kept as you had it.
    - VCM_SOLAR / VCM_BAT default to 1.42V (your bench-measured value).
      Recalibrate with 'zv' (both AMC1311 IN pins grounded) if you rebuild.
    - Current: QUIESCENT_VOLTAGE is a single SHARED value (both channels
      averaged together), default 1.56V (your measured value, not the
      theoretical VCC/2). Recalibrate with 'zc' (zero current on both
      channels) if conditions change.

  MPPT ALGORITHM: Perturb & Observe (P&O)
    Every PERTURB_INTERVAL, nudges duty by perturbStep and checks if PV power
    (V x I) went up or down. If up, keep going the same direction; if down,
    reverse direction. Hill-climbs toward the panel's max power point.

  SAFETY:
    - PWM starts DISABLED at boot. Type 'on' to enable output.
    - Duty always clamped to DUTY_MIN..DUTY_MAX (5-70%), auto or manual.
    - OVERCURRENT_LIMIT_A trips PWM off and drops out of auto mode.
    - BATTERY_MAX_VOLTAGE stops P&O from pushing duty higher once reached.
    - 'd XX' manual command always switches OUT of auto mode first.
  ============================================================================
*/

// ================== PWM PINS ==================
#define PWM_H_PIN     19
#define PWM_L_PIN     18

// ================== VOLTAGE SENSE PINS (AMC1311, ADC1) ==================
#define SOLAR_V_PIN   34   // ADC1_CH6
#define BAT_V_PIN     35   // ADC1_CH7

// ================== CURRENT SENSE PINS (ACS758, ADC2) ==================
#define SOLAR_I_PIN   13   // ADC2_CH4 -- WiFi conflict, see note above
#define BAT_I_PIN     12   // ADC2_CH5 -- WiFi conflict, see note above

// ================== PWM SETTINGS ==================
#define LEDC_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_SEL        LEDC_TIMER_0
#define LEDC_CH_H              LEDC_CHANNEL_0
#define LEDC_CH_L              LEDC_CHANNEL_1
#define LEDC_RESOLUTION_BITS  LEDC_TIMER_10_BIT

static const uint32_t PWM_PERIOD_COUNTS = 1 << 10;
static float PWM_FREQ_HZ  = 30000.0f;   // 30 kHz
static float DEAD_TIME_US = 2.0f;
static int   currentDuty  = 40;         // Start LOW, raise slowly
static const int DUTY_MIN = 5;
static const int DUTY_MAX = 70;         // Hard ceiling
static bool  pwmEnabled   = false;      // Output OFF until 'on' command

// ================== MPPT (PERTURB & OBSERVE) CONFIG ==================
bool  mpptAutoEnabled        = false;   // Auto P&O disabled by default
int   perturbStep            = 1;       // Duty step size (%) per P&O iteration
int   perturbDirection       = 1;       // +1 = increasing duty, -1 = decreasing
float lastPower               = 0.0f;   // Previous PV power sample
unsigned long lastPerturbTime = 0;
const unsigned long PERTURB_INTERVAL = 1000;   // P&O step every 1s

// ================== BATTERY PROTECTION ==================
const float BATTERY_MAX_VOLTAGE = 14.4f;   // Tune to your battery chemistry/capacity
const float BATTERY_MIN_VOLTAGE = 10.5f;   // Informational -- deep discharge floor

// ================== AMC1311 VOLTAGE SENSE CONFIG (your calibrated values) ==================
const float GAIN_FACTOR = 0.5f;    // VOUTP-only readout
float VCM_SOLAR = 1.42f;           // Calibrate with 'zv'
float VCM_BAT   = 1.42f;           // Calibrate with 'zv'

const float ALPHA_PV  = 1.0f / 31.0f;   // PV divider ratio
const float ALPHA_BAT = 1.0f / 31.0f;   // BAT divider ratio
const float SCALE_PV  = 1.0f / (ALPHA_PV);    // No extra GAIN_FACTOR here -- already applied in v_in
const float SCALE_BAT = 1.0f / (ALPHA_BAT);

const float VREF      = 3.30f;
const int   V_SAMPLES = 16;

// ================== ACS758 CURRENT SENSE CONFIG (your calibrated values) ==================
const float VCC_SENSOR = 3.30f;
const float SENSITIVITY_V_PER_A = 0.020f * (VCC_SENSOR / 5.0f);   // ~0.0132 V/A estimate
float QUIESCENT_VOLTAGE = 1.56f;    // Your measured value -- recalibrate with 'zc'
const int I_SAMPLES = 32;

// EMA smoothing for current display
float solarCurrentFiltered = 0.0f;
float batCurrentFiltered   = 0.0f;
const float EMA_ALPHA = 0.2f;

// ================== SAFETY ==================
const float OVERCURRENT_LIMIT_A = 5.0f;   // Tune to your fuse/setup

// ================== TELEMETRY TIMER ==================
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 500;

// ================== FUNCTION DECLARATIONS ==================
void checkErr(const char* what, esp_err_t err);
void pwmOff();
void updatePWM(int duty);
void printStatus();
float readAveragedAdc(int pin, int samples);
float readSolarVoltage();
float readBatVoltage();
float readSolarCurrent();
float readBatCurrent();
void calibrateCurrentZero();
void calibrateVCM();
void checkOvercurrent(float iSolar, float iBat);
void perturbAndObserve(float pvVoltage, float pvCurrent, float batVoltage);
void handleSerialCommands();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== ESP32 MPPT Solar Charge Controller (P&O) ===");
    Serial.println(">>> PWM starts DISABLED. Type 'on' to enable output. <<<");
    Serial.println(">>> MPPT auto mode is OFF by default -- type 'auto on' to enable P&O. <<<\n");

    analogReadResolution(12);
    analogSetPinAttenuation(SOLAR_V_PIN, ADC_11db);
    analogSetPinAttenuation(BAT_V_PIN,   ADC_11db);
    analogSetPinAttenuation(SOLAR_I_PIN, ADC_11db);
    analogSetPinAttenuation(BAT_I_PIN,   ADC_11db);

    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = LEDC_MODE;
    timer_conf.duty_resolution = LEDC_RESOLUTION_BITS;
    timer_conf.timer_num       = LEDC_TIMER_SEL;
    timer_conf.freq_hz         = (uint32_t)PWM_FREQ_HZ;
    timer_conf.clk_cfg         = LEDC_USE_APB_CLK;
    checkErr("timer_config", ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_h = {};
    ch_h.gpio_num   = PWM_H_PIN;
    ch_h.speed_mode = LEDC_MODE;
    ch_h.channel    = LEDC_CH_H;
    ch_h.timer_sel  = LEDC_TIMER_SEL;
    checkErr("ch_h", ledc_channel_config(&ch_h));

    ledc_channel_config_t ch_l = ch_h;
    ch_l.gpio_num = PWM_L_PIN;
    ch_l.channel  = LEDC_CH_L;
    checkErr("ch_l", ledc_channel_config(&ch_l));

    pwmOff();   // Ensure output OFF at boot
    printStatus();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    handleSerialCommands();

    if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
        lastTelemetryTime = millis();

        float pvVoltage  = readSolarVoltage();
        float batVoltage = readBatVoltage();
        float iSolarRaw  = readSolarCurrent();
        float iBatRaw    = readBatCurrent();

        solarCurrentFiltered = (EMA_ALPHA * iSolarRaw) + ((1.0f - EMA_ALPHA) * solarCurrentFiltered);
        batCurrentFiltered   = (EMA_ALPHA * iBatRaw)   + ((1.0f - EMA_ALPHA) * batCurrentFiltered);

        checkOvercurrent(iSolarRaw, iBatRaw);

        float pvPower = pvVoltage * iSolarRaw;

        Serial.printf("[%s|%s] Duty=%d%% | PV=%.2fV %.2fA (%.1fW) | BAT=%.2fV %.2fA(%.2fA)\n",
                      pwmEnabled ? "ON " : "OFF",
                      mpptAutoEnabled ? "AUTO" : "MAN ",
                      currentDuty, pvVoltage, iSolarRaw, pvPower,
                      batVoltage, iBatRaw, batCurrentFiltered);
    }

    // MPPT Perturb & Observe -- own slower interval so the buck loop has
    // time to settle after each duty change before judging the result
    if (mpptAutoEnabled && pwmEnabled && (millis() - lastPerturbTime >= PERTURB_INTERVAL)) {
        lastPerturbTime = millis();
        float pvVoltage  = readSolarVoltage();
        float iSolarRaw  = readSolarCurrent();
        float batVoltage = readBatVoltage();
        perturbAndObserve(pvVoltage, iSolarRaw, batVoltage);
    }
}

// ============================================================================
// PWM FUNCTIONS
// ============================================================================
void checkErr(const char* what, esp_err_t err) {
    if (err != ESP_OK) {
        Serial.printf("ERROR in %s: %s\n", what, esp_err_to_name(err));
        while (true) { delay(1000); }
    }
}

void pwmOff() {
    ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CH_H, 0, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CH_H);
    ledc_set_duty_with_hpoint(LEDC_MODE, LEDC_CH_L, 0, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CH_L);
    pwmEnabled = false;
    Serial.println(">>> PWM OUTPUT DISABLED <<<");
}

void updatePWM(int duty) {
    if (duty < DUTY_MIN) duty = DUTY_MIN;
    if (duty > DUTY_MAX) duty = DUTY_MAX;

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
    pwmEnabled  = true;
}

// ============================================================================
// VOLTAGE SENSE (AMC1311, single-ended VOUTP-only, your calibrated formulas)
// ============================================================================
float readAveragedAdc(int pin, int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delayMicroseconds(150);
    }
    float raw = (float)sum / samples;
    return raw * (VREF / 4095.0f);
}

float readSolarVoltage() {
    float v_adc = readAveragedAdc(SOLAR_V_PIN, V_SAMPLES);
    float v_in  = (v_adc - VCM_SOLAR) / GAIN_FACTOR;
    if (v_in < 0) v_in = 0;
    return v_in * SCALE_PV;
}

float readBatVoltage() {
    // NOTE: +0.1V correction kept exactly as you tuned it -- solar channel
    // does not get this offset, only battery.
    float v_adc = readAveragedAdc(BAT_V_PIN, V_SAMPLES) + 0.1f;
    float v_in  = (v_adc - VCM_BAT) / GAIN_FACTOR;
    if (v_in < 0) v_in = 0;
    return v_in * SCALE_BAT;
}

void calibrateVCM() {
    Serial.println("\n[Calibrating VCM... ensure BOTH AMC1311 IN pins are grounded (zero input)]");
    delay(200);
    float vSolar = readAveragedAdc(SOLAR_V_PIN, V_SAMPLES);
    float vBat   = readAveragedAdc(BAT_V_PIN, V_SAMPLES);
    VCM_SOLAR = vSolar;
    VCM_BAT   = vBat;
    Serial.printf("VCM_SOLAR updated to: %.4f V\n", vSolar);
    Serial.printf("VCM_BAT updated to:   %.4f V\n\n", vBat);
}

// ============================================================================
// CURRENT SENSE (ACS758, direct connection, shared quiescent voltage)
// ============================================================================
float readSolarCurrent() {
    float v = readAveragedAdc(SOLAR_I_PIN, I_SAMPLES);
    return (v - QUIESCENT_VOLTAGE) / SENSITIVITY_V_PER_A;
}

float readBatCurrent() {
    float v = readAveragedAdc(BAT_I_PIN, I_SAMPLES);
    return (v - QUIESCENT_VOLTAGE) / SENSITIVITY_V_PER_A;
}

void calibrateCurrentZero() {
    if (pwmEnabled) {
        Serial.println("Refusing to calibrate while PWM is running -- turn off first ('off').");
        return;
    }
    Serial.println("\n[Calibrating current-sense zero offset... ensure NO current is flowing]");
    delay(200);
    float v_solar = readAveragedAdc(SOLAR_I_PIN, I_SAMPLES);
    float v_bat   = readAveragedAdc(BAT_I_PIN, I_SAMPLES);
    Serial.printf("Measured quiescent -> Solar: %.4f V | Bat: %.4f V\n", v_solar, v_bat);
    QUIESCENT_VOLTAGE = (v_solar + v_bat) / 2.0f;
    Serial.printf("QUIESCENT_VOLTAGE updated to %.4f V\n\n", QUIESCENT_VOLTAGE);
}

// ============================================================================
// SAFETY
// ============================================================================
void checkOvercurrent(float iSolar, float iBat) {
    if (pwmEnabled && (fabs(iSolar) > OVERCURRENT_LIMIT_A || fabs(iBat) > OVERCURRENT_LIMIT_A)) {
        Serial.println("!!! OVERCURRENT TRIP -- SHUTTING DOWN PWM !!!");
        Serial.printf("Solar: %.2f A | Bat: %.2f A | Limit: %.2f A\n", iSolar, iBat, OVERCURRENT_LIMIT_A);
        pwmOff();
        mpptAutoEnabled = false;   // Require manual re-enable after inspection
    }
}

// ============================================================================
// PERTURB & OBSERVE MPPT ALGORITHM
// ============================================================================
void perturbAndObserve(float pvVoltage, float pvCurrent, float batVoltage) {
    // Battery protection overrides MPPT -- never push duty higher once the
    // battery reaches its charge ceiling, regardless of what P&O wants.
    if (batVoltage >= BATTERY_MAX_VOLTAGE) {
        if (currentDuty > DUTY_MIN) {
            updatePWM(currentDuty - perturbStep);
            Serial.printf("[MPPT] Battery at ceiling (%.2fV) -- backing off duty to %d%%\n", batVoltage, currentDuty);
        }
        return;
    }

    float power = pvVoltage * pvCurrent;

    if (power > lastPower) {
        // Keep going the same direction
    } else {
        perturbDirection = -perturbDirection;
    }

    int newDuty = currentDuty + (perturbDirection * perturbStep);
    updatePWM(newDuty);

    Serial.printf("[MPPT] P=%.2fW (prev %.2fW) -> duty %d%%\n", power, lastPower, currentDuty);

    lastPower = power;
}

// ============================================================================
// STATUS
// ============================================================================
void printStatus() {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.printf("PWM            : %s\n", pwmEnabled ? "ENABLED" : "DISABLED");
    Serial.printf("MPPT Mode      : %s\n", mpptAutoEnabled ? "AUTO (P&O)" : "MANUAL");
    Serial.printf("Duty           : %d%%\n", currentDuty);
    Serial.printf("Perturb Step   : %d%%\n", perturbStep);
    Serial.printf("VCM_SOLAR      : %.4f V | VCM_BAT: %.4f V\n", VCM_SOLAR, VCM_BAT);
    Serial.printf("Current Zero   : %.4f V (shared)\n", QUIESCENT_VOLTAGE);
    Serial.printf("Battery Ceiling: %.2f V\n", BATTERY_MAX_VOLTAGE);
    Serial.println("Commands:");
    Serial.println("  on         -> enable PWM output at current duty");
    Serial.println("  off        -> disable PWM output (safe stop)");
    Serial.println("  d XX       -> set duty manually (e.g. d 60), switches to MANUAL mode");
    Serial.println("  auto on    -> enable MPPT Perturb & Observe");
    Serial.println("  auto off   -> disable MPPT, return to manual duty control");
    Serial.println("  zc         -> calibrate current-sense zero (PWM off, no current flowing)");
    Serial.println("  zv         -> calibrate voltage-sense VCM (both IN pins grounded)");
    Serial.println("  s          -> show this status");
    Serial.println("======================\n");
}

// ============================================================================
// SERIAL COMMAND PARSER
// ============================================================================
void handleSerialCommands() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("on")) {
            updatePWM(currentDuty);
            Serial.println(">> PWM enabled.");
        }
        else if (input.equalsIgnoreCase("off")) {
            pwmOff();
        }
        else if (input.equalsIgnoreCase("auto on")) {
            mpptAutoEnabled = true;
            lastPower = 0.0f;   // Reset so the first P&O cycle doesn't compare against a stale value
            Serial.println(">> MPPT auto mode ENABLED (Perturb & Observe).");
        }
        else if (input.equalsIgnoreCase("auto off")) {
            mpptAutoEnabled = false;
            Serial.println(">> MPPT auto mode DISABLED -- manual duty control active.");
        }
        else if (input.startsWith("d ") || input.startsWith("D ")) {
            int newDuty = input.substring(2).toInt();
            mpptAutoEnabled = false;   // Manual override drops out of auto mode
            updatePWM(newDuty);
            Serial.printf(">> Manual duty set to: %d%% (auto mode disabled)\n", currentDuty);
        }
        else if (input.equalsIgnoreCase("zc")) {
            calibrateCurrentZero();
        }
        else if (input.equalsIgnoreCase("zv")) {
            calibrateVCM();
        }
        else if (input.equalsIgnoreCase("s")) {
            printStatus();
        }
    }
}