# Everything the HSA wrist reports, one entry per axis (index = ODrive node id = HSA index),
# sampled by wrist_state_publisher at a fixed rate. The ODrive CAN messages carry no stamp, so
# the header is the publisher's clock and each *_age_s is how old that source's newest sample
# was at publish time (NaN before the first sample). Arrays are NaN / 0 for an axis that has not
# reported yet.
std_msgs/Header header

# hsa_odrive_control: measured extension angle per HSA (deg, its own conversion with the software
# zero) and the last commanded extension angle (deg; NaN until the first command).
float32[] extension_deg
float32[] command_deg
float32 extension_age_s
float32 command_age_s

# odrive_can ControllerStatus per axis
float32[] motor_pos_turns
float32[] motor_vel_turns_s
float32[] torque_target_nm
float32[] torque_estimate_nm
float32[] iq_setpoint_a
float32[] iq_measured_a
uint8[] axis_state
uint32[] controller_errors
float32[] controller_age_s

# odrive_can ODriveStatus per axis
float32[] bus_voltage_v
float32[] bus_current_a
float32[] fet_temperature_c
float32[] motor_temperature_c
uint32[] odrive_errors
uint32[] disarm_reason
float32[] odrive_age_s
