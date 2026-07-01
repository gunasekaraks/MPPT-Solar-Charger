#include "mppt_controller.h"

namespace {
constexpr float kDutyMin = 0.05f;
constexpr float kDutyMax = 0.95f;
constexpr float kDutyStep = 0.01f;

MpptState state = {0.0f, 0.0f, 0.0f, 0.0f, 0.50f};
float previous_power = 0.0f;
float previous_voltage = 0.0f;
bool first_update = true;
}  // namespace

void mppt_controller_init() {
  mppt_controller_reset();
}

void mppt_controller_reset() {
  state = {0.0f, 0.0f, 0.0f, 0.0f, 0.50f};
  previous_power = 0.0f;
  previous_voltage = 0.0f;
  first_update = true;
}

void mppt_controller_set_measurements(float solar_voltage,
                                      float solar_current,
                                      float battery_voltage) {
  state.solar_voltage = solar_voltage;
  state.solar_current = solar_current;
  state.battery_voltage = battery_voltage;
  state.power_watts = solar_voltage * solar_current;
}

const MpptState &mppt_controller_update() {
  const float current_power = state.power_watts;
  const float voltage_delta = state.solar_voltage - previous_voltage;
  const float power_delta = current_power - previous_power;

  if (first_update) {
    first_update = false;
  } else if (power_delta > 0.0f) {
    if (voltage_delta > 0.0f) {
      state.duty_cycle -= kDutyStep;
    } else {
      state.duty_cycle += kDutyStep;
    }
  } else if (power_delta < 0.0f) {
    if (voltage_delta > 0.0f) {
      state.duty_cycle += kDutyStep;
    } else {
      state.duty_cycle -= kDutyStep;
    }
  }

  if (state.duty_cycle < kDutyMin) {
    state.duty_cycle = kDutyMin;
  } else if (state.duty_cycle > kDutyMax) {
    state.duty_cycle = kDutyMax;
  }

  previous_power = current_power;
  previous_voltage = state.solar_voltage;
  return state;
}

const MpptState &mppt_controller_get_state() {
  return state;
}