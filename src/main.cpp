#include <Arduino.h>

#include "lcd_display.h"
#include "mppt_controller.h"

namespace {
unsigned long last_update_ms = 0;
float simulated_voltage = 18.0f;
float simulated_current = 2.0f;
float simulated_battery = 12.4f;
}  // namespace

void setup() {
  mppt_controller_init();
  lcd_display_init();
}

void loop() {
  if (millis() - last_update_ms >= 1000UL) {
    last_update_ms = millis();

    simulated_voltage += 0.1f;
    if (simulated_voltage > 20.0f) {
      simulated_voltage = 17.5f;
    }

    simulated_current = 1.5f + (simulated_voltage - 17.5f) * 0.3f;
    if (simulated_current < 0.0f) {
      simulated_current = 0.0f;
    }

    simulated_battery += 0.01f;

    mppt_controller_set_measurements(simulated_voltage, simulated_current, simulated_battery);
    const MpptState &state = mppt_controller_update();

    char line1[24];
    char line2[24];

    snprintf(line1, sizeof(line1), "PV %.1fV Bat %.1fV", state.solar_voltage, state.battery_voltage);
    snprintf(line2, sizeof(line2), "P %.1fW D %.0f%%", state.power_watts, state.duty_cycle * 100.0f);
    lcd_display_show_text(line1, line2);
  }
}