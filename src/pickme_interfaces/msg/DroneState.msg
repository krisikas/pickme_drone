# Drone state reported by pickme_commander
# Aggregates key PX4 telemetry into a single convenient topic

uint8 STATE_DISARMED=0
uint8 STATE_ARMED=1
uint8 STATE_TAKING_OFF=2
uint8 STATE_HOVERING=3
uint8 STATE_IN_MISSION=4
uint8 STATE_LANDING=5
uint8 STATE_LANDED=6

std_msgs/Header header

uint8 state
bool offboard_active
float32 battery_voltage
float32 altitude_agl          # altitude above ground level (meters)
geometry_msgs/Pose pose       # current pose in local NED frame
geometry_msgs/Twist velocity  # current velocity
