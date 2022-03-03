// ----------------------------------------------------------------------------
// TEENSY40.cpp
//
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TEENSY40.h"


#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY40)
namespace mouse_reach_linear_controller
{
namespace constants
{
const modular_server::HardwareInfo hardware_info =
{
  .name_ptr=&hardware_name,
  .part_number=2035,
  .version_major=1,
  .version_minor=2,
};

// Pins
const size_t signal_a_pin_number = 22;

const size_t signal_b_pin_number = 4;

const size_t signal_c_pin_number = 3;

const size_t signal_d_pin_number = 2;

const size_t power_switch_pin_number = 5;

// Units

// Properties
const long steps_per_position_units_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  steps_per_position_units_0_element_default,
  steps_per_position_units_1_2_element_default,
  steps_per_position_units_1_2_element_default,
};

const long velocity_max_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  velocity_max_0_element_default,
  velocity_max_1_2_element_default,
  velocity_max_1_2_element_default,
};

const long velocity_min_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  velocity_min_0_element_default,
  velocity_min_1_2_element_default,
  velocity_min_1_2_element_default,
};

const long acceleration_max_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  acceleration_max_0_element_default,
  acceleration_max_1_2_element_default,
  acceleration_max_1_2_element_default,
};

const long home_velocity_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  home_velocity_0_element_default,
  home_velocity_1_2_element_default,
  home_velocity_1_2_element_default,
};

const bool right_switches_enabled_default[step_dir_controller::constants::CONTROLLER_COUNT_MAX] =
{
  right_switches_enabled_element_default,
};

const bool right_switch_stop_enabled_default[step_dir_controller::constants::CHANNEL_COUNT_MAX] =
{
  right_switch_stop_enabled_element_default,
  right_switch_stop_enabled_element_default,
  right_switch_stop_enabled_element_default,
};

const bool invert_driver_direction_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  invert_driver_direction_0_element_default,
  invert_driver_direction_1_2_element_default,
  invert_driver_direction_1_2_element_default,
};

const long run_current_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  run_current_0_element_default,
  run_current_1_2_element_default,
  run_current_1_2_element_default,
};

const long pwm_offset_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  pwm_offset_0_element_default,
  pwm_offset_1_2_element_default,
  pwm_offset_1_2_element_default,
};

const long pwm_gradient_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  pwm_gradient_0_element_default,
  pwm_gradient_1_2_element_default,
  pwm_gradient_1_2_element_default,
};

const bool cool_step_enabled_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  cool_step_enabled_0_element_default,
  cool_step_enabled_1_2_element_default,
  cool_step_enabled_1_2_element_default,
};

const long stage_position_min_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  stage_position_min_element_default,
  stage_position_min_element_default,
  stage_position_min_element_default,
};

const long stage_position_max_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  stage_position_max_0_element_default,
  stage_position_max_1_2_element_default,
  stage_position_max_1_2_element_default,
};

const long buzz_position_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  buzz_position_0_element_default,
  buzz_position_1_element_default,
  buzz_position_2_element_default,
};

const long load_position_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  load_position_0_element_default,
  load_position_1_element_default,
  load_position_2_element_default,
};

const long next_deliver_position_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  next_deliver_position_0_element_default,
  next_deliver_position_1_element_default,
  next_deliver_position_2_element_default,
};

const long next_dispense_position_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  next_dispense_position_0_element_default,
  next_dispense_position_1_element_default,
  next_dispense_position_2_element_default,
};

const long dispense_velocity_default[stepper_controller::constants::CHANNEL_COUNT_MAX] =
{
  dispense_velocity_0_element_default,
  dispense_velocity_1_2_element_default,
  dispense_velocity_1_2_element_default,
};

// Parameters

// Functions

// Callbacks

// Errors
}
}

#endif
