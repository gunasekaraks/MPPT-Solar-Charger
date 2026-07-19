/*
  ============================================================================
  ESP32 BUCK CHARGER - MANUAL DUTY CONTROL
  Input: 25V Power Supply → 12V Battery
  30 kHz with live Serial duty adjustment
  ============================================================================
*/

#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_err.h"

// Pins
#define PWM_H_PIN   19
#define PWM_L_PIN   18

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
static int currentDuty    = 40;         // Safe starting duty for 25V → 12V

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

void printStatus() {
  Serial.println("\n=== STATUS ===");
  Serial.print("Frequency : 30 kHz");
  Serial.print("\nDuty      : ");
  Serial.print(currentDuty);
  Serial.println("%");
  Serial.println("Commands:");
  Serial.println("  d XX   → set duty (e.g. d 48)");
  Serial.println("  s      → show status");
  Serial.println("==============\n");
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n=== 25V → 12V Battery Charger (Manual Control) ===");
  Serial.println(">>> Start low and increase duty slowly while watching current <<<\n");

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
  }

  // Periodic status
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 20000) {
    Serial.print("[Running] Duty = ");
    Serial.print(currentDuty);
    Serial.println("% | 30kHz");
    lastPrint = millis();
  }
}
