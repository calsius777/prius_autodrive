builtin_interfaces/Time stamp

# Receiver status
uint64 rx_total_count
uint64 rx_error_count

# Validity flags
bool has_speed
bool has_steering
bool has_wheel_speeds
bool has_brake
bool has_gas
bool has_wire_status
bool has_turn_signal
bool has_abs

# 0x0B4 SPEED
uint16 speed_raw
float64 speed_kmh
float64 speed_mps

# 0x025 STEER_ANGLE_SENSOR_D
int16 steering_raw
float64 steering_wheel_deg
float64 steering_wheel_rad

# 0x0AA WHEEL_SPEEDS
# Current mapping inferred from road-test data:
# wheel_01=FR, wheel_23=FL, wheel_45=RR, wheel_67=RL.
uint16 wheel_01_raw
uint16 wheel_23_raw
uint16 wheel_45_raw
uint16 wheel_67_raw
float64 wheel_01_kmh
float64 wheel_23_kmh
float64 wheel_45_kmh
float64 wheel_67_kmh
float64 wheel_fr_kmh
float64 wheel_fl_kmh
float64 wheel_rr_kmh
float64 wheel_rl_kmh
float64 wheel_fr_mps
float64 wheel_fl_mps
float64 wheel_rr_mps
float64 wheel_rl_mps

# 0x226 BRAKE_MODULE
bool brake_pressed
uint16 brake_pressure_candidate
uint8 brake_position_raw
uint8 brake_status_byte4

# 0x245 GAS_PEDAL
uint8 gas_pedal_raw
uint8 engine_status_code
string engine_status_text

# 0x203 WIRE_CONTROLLER_STATUS
bool wire_control_enabled
uint8 wire_status_byte7
uint8 wire_status_byte4

# 0x614 STEERING_LEVERS
uint8 turn_signal_code
string turn_signal_text
bool turn_blink_bit

# 0x1AA ABS_CONTROL, candidate decoding
bool abs_fl
bool abs_fr
bool abs_rl
bool abs_rr
