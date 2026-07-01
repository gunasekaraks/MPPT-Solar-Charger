#ifndef MPPT_CONTROLLER_H
#define MPPT_CONTROLLER_H

#include <Arduino.h>

struct MpptState {
  float solar_voltage;
  float solar_current;
  float battery_voltage;
  float power_watts;
  float duty_cycle;
};

void mppt_controller_init();
void mppt_controller_reset();
void mppt_controller_set_measurements(float solar_voltage,
                                      float solar_current,
                                      float battery_voltage);
const MpptState &mppt_controller_update();
const MpptState &mppt_controller_get_state();

#endif