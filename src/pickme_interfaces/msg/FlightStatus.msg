# PickmeDrone Offboard Flight Status & Telemetry
std_msgs/Header header

# High-level Flight State Machine
# DISARMED, ARMING, ARMED, TAKEOFF, NAVIGATING, HOVERING, LANDING, EMERGENCY
string state

# System health & connectivity
bool connected
bool armed
bool offboard_mode
bool target_reached

# PX4 Navigation state
uint8 nav_state
string nav_state_str

# Estimated kinematics in odom (ENU) frame
geometry_msgs/Point current_position
geometry_msgs/Vector3 current_velocity
float32 current_yaw

# Active navigation setpoint
geometry_msgs/Point target_position
float32 target_yaw
float32 target_speed
float32 remaining_distance

# Power & Diagnostics
float32 battery_voltage
float32 battery_percentage
string last_error
