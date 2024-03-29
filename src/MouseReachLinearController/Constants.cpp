// ----------------------------------------------------------------------------
// Constants.cpp
//
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Constants.h"


namespace mouse_reach_linear_controller
{
namespace constants
{
CONSTANT_STRING(device_name,"mouse_reach_linear_controller");

CONSTANT_STRING(firmware_name,"MouseReachLinearController");
// Use semantic versioning http://semver.org/
const modular_server::FirmwareInfo firmware_info =
{
  .name_ptr=&firmware_name,
  .version_major=1,
  .version_minor=0,
  .version_patch=2,
};

CONSTANT_STRING(hardware_name,"mouse_reach_controller");

CONSTANT_STRING(state_string,"state");
CONSTANT_STRING(state_assay_not_started_string,"ASSAY_NOT_STARTED");
CONSTANT_STRING(state_assay_started_string,"ASSAY_STARTED");
CONSTANT_STRING(state_homing_string,"HOMING");
CONSTANT_STRING(state_move_to_buzz_string,"MOVE_TO_BUZZ");
CONSTANT_STRING(state_moving_to_buzz_string,"MOVING_TO_BUZZ");
CONSTANT_STRING(state_move_to_load_string,"MOVE_TO_LOAD");
CONSTANT_STRING(state_moving_to_load_string,"MOVING_TO_LOAD");
CONSTANT_STRING(state_wait_at_load_string,"WAIT_AT_LOAD");
CONSTANT_STRING(state_waiting_at_load_string,"WAITING_AT_LOAD");
CONSTANT_STRING(state_move_to_next_deliver_string,"MOVE_TO_NEXT_DELIVER");
CONSTANT_STRING(state_moving_to_next_deliver_string,"MOVING_TO_NEXT_DELIVER");
CONSTANT_STRING(state_ready_to_dispense_string,"READY_TO_DISPENSE");
CONSTANT_STRING(state_wait_to_dispense_string,"WAIT_TO_DISPENSE");
CONSTANT_STRING(state_waiting_to_dispense_string,"WAITING_TO_DISPENSE");
CONSTANT_STRING(state_move_to_dispense_string,"MOVE_TO_DISPENSE");
CONSTANT_STRING(state_moving_to_dispense_string,"MOVING_TO_DISPENSE");
CONSTANT_STRING(state_wait_to_return_string,"WAIT_TO_RETURN");
CONSTANT_STRING(state_waiting_to_return_string,"WAITING_TO_RETURN");
CONSTANT_STRING(state_assay_finished_string,"ASSAY_FINISHED");
CONSTANT_STRING(state_buzz_string,"BUZZ");
CONSTANT_STRING(state_buzzing_string,"BUZZING");
CONSTANT_STRING(state_tap_string,"TAP");
CONSTANT_STRING(state_tapping_string,"TAPPING");

const long seconds_per_minute = 60;
const long milliseconds_per_second = 1000;
const long milliseconds_per_minute = milliseconds_per_second*seconds_per_minute;

CONSTANT_STRING(set_value_string,"setValue");

// Pins
CONSTANT_STRING(signal_a_pin_name,"signal_a");

CONSTANT_STRING(signal_b_pin_name,"signal_b");

CONSTANT_STRING(signal_c_pin_name,"signal_c");

CONSTANT_STRING(signal_d_pin_name,"signal_d");

CONSTANT_STRING(power_switch_pin_name,"power_switch");

// Units
CONSTANT_STRING(mm_units,"mm");
CONSTANT_STRING(ms_units,"ms");
CONSTANT_STRING(seconds_units,"s");
CONSTANT_STRING(minutes_units,"min");
CONSTANT_STRING(hz_units,"Hz");
CONSTANT_STRING(percent_units,"%");

const long tone_frequency_min = 200;
const long tone_frequency_max = 22000;

// Properties
// microsteps/mm = (256microsteps/step)/(0.102mm/step)
const long steps_per_position_units_0_element_default = 2510;
// microsteps/mm = (200steps/rev)*(256microsteps/step)/((0.250in/rev)*(25.4mm/in))
const long steps_per_position_units_1_2_element_default = 8063;

const long velocity_max_0_element_default = 40;
const long velocity_max_1_2_element_default = 40;

const long velocity_min_0_element_default = 2;
const long velocity_min_1_2_element_default = 2;

const long acceleration_max_0_element_default = 40;
const long acceleration_max_1_2_element_default = 40;

const long home_velocity_0_element_default = -10;
const long home_velocity_1_2_element_default = -10;

const bool right_switches_enabled_element_default = false;

const bool right_switch_stop_enabled_element_default = false;

const bool invert_driver_direction_0_element_default = false;
const bool invert_driver_direction_1_2_element_default = false;

const long run_current_0_element_default = 100;
const long run_current_1_2_element_default = 100;

const long pwm_offset_0_element_default = 150;
const long pwm_offset_1_2_element_default = 40;

const long pwm_gradient_0_element_default = 20;
const long pwm_gradient_1_2_element_default = 5;

const bool cool_step_enabled_0_element_default = false;
const bool cool_step_enabled_1_2_element_default = false;

const long stage_position_min_element_default = 0;

const long stage_position_max_0_element_default = 20;
const long stage_position_max_1_2_element_default = 50;

const long buzz_position_0_element_default = 10;
const long buzz_position_1_element_default = 25;
const long buzz_position_2_element_default = 35;

const long load_position_0_element_default = 1;
const long load_position_1_element_default = 25;
const long load_position_2_element_default = 35;

const long next_deliver_position_0_element_default = 17;
const long next_deliver_position_1_element_default = 26;
const long next_deliver_position_2_element_default = 34;

const long next_dispense_position_0_element_default = 13;
const long next_dispense_position_1_element_default = 21;
const long next_dispense_position_2_element_default = 44;

CONSTANT_STRING(buzz_position_property_name,"buzzPosition");

CONSTANT_STRING(load_position_property_name,"loadPosition");

CONSTANT_STRING(next_deliver_position_property_name,"nextDeliverPosition");

CONSTANT_STRING(next_dispense_position_property_name,"nextDispensePosition");

CONSTANT_STRING(position_property_name,"position");
CONSTANT_STRING(position_left,"LEFT");
CONSTANT_STRING(position_right,"RIGHT");
modular_server::SubsetMemberType position_subset[POSITION_SUBSET_LENGTH] =
{
  {.cs_ptr=&position_left},
  {.cs_ptr=&position_right},
};
const ConstantString * const position_ptr_default = &position_left;

CONSTANT_STRING(position_tone_frequency_property_name,"positionToneFrequency");
const long position_tone_frequency_default = 3000;

CONSTANT_STRING(position_tone_volume_property_name,"positionToneVolume");
const long position_tone_volume_default = 25;

CONSTANT_STRING(position_tone_delay_property_name,"positionToneDelay");
const double position_tone_delay_min = 0.0;
const double position_tone_delay_max = 100.0;
const double position_tone_delay_default = 0.0;

CONSTANT_STRING(position_tone_duration_property_name,"positionToneDuration");
const double position_tone_duration_min = 0.001;
const double position_tone_duration_max = 10.0;
const double position_tone_duration_default = 0.2;

CONSTANT_STRING(dispense_delay_property_name,"dispenseDelay");
const double dispense_delay_min = 0.0;
const double dispense_delay_max = 100.0;
const double dispense_delay_default = 0.0;

CONSTANT_STRING(return_delay_min_property_name,"returnDelayMin");
const double return_delay_min = 1.0;
const double return_delay_max = 3600.0;
const double return_delay_min_default = 4.0;

CONSTANT_STRING(return_delay_max_property_name,"returnDelayMax");
const double return_delay_max_default = 4.0;

CONSTANT_STRING(buzz_period_property_name,"buzzPeriod");
const long buzz_period_min = 10;
const long buzz_period_max = 100000;
const long buzz_period_default = 100;

CONSTANT_STRING(buzz_on_duration_property_name,"buzzOnDuration");
const long buzz_on_duration_min = 1;
const long buzz_on_duration_max = 10000;
const long buzz_on_duration_default = 1;

CONSTANT_STRING(buzz_count_property_name,"buzzCount");
const long buzz_count_min = 1;
const long buzz_count_max = 5;
const long buzz_count_default = 1;

CONSTANT_STRING(wait_at_load_duration_property_name,"waitAtLoadDuration");
const long wait_at_load_duration_min = 1;
const long wait_at_load_duration_max = 10000;
const long wait_at_load_duration_default = 1000;

CONSTANT_STRING(tap_period_property_name,"tapPeriod");
const long tap_period_min = 10;
const long tap_period_max = 100000;
const long tap_period_default = 500;

CONSTANT_STRING(tap_on_duration_property_name,"tapOnDuration");
const long tap_on_duration_min = 1;
const long tap_on_duration_max = 10000;
const long tap_on_duration_default = 40;

CONSTANT_STRING(tap_count_property_name,"tapCount");
const long tap_count_min = 1;
const long tap_count_max = 5;
const long tap_count_default = 5;

CONSTANT_STRING(dispense_velocity_property_name,"dispenseVelocity");
const long dispense_velocity_min = 1;
const long dispense_velocity_max = 100;
const long dispense_velocity_0_element_default = 80;
const long dispense_velocity_1_2_element_default = 40;

// Parameters

// Functions
CONSTANT_STRING(get_assay_status_function_name,"getAssayStatus");
CONSTANT_STRING(play_position_tone_function_name,"playPositionTone");
CONSTANT_STRING(buzz_function_name,"buzz");

// Callbacks
CONSTANT_STRING(start_assay_callback_name,"startAssay");
CONSTANT_STRING(dispense_callback_name,"dispense");
CONSTANT_STRING(abort_callback_name,"abort");

// Errors
}
}
