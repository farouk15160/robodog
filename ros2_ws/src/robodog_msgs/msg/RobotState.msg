# The single aggregate telemetry topic. One message per control cycle carries
# everything the GUI, the logger and an operator need, so no consumer has to
# time-align several topics to render a consistent picture.

uint8 STATE_INIT      = 0
uint8 STATE_IDLE      = 1   # powered, joints disabled
uint8 STATE_READY     = 2   # joints enabled, holding position
uint8 STATE_STANDING  = 3
uint8 STATE_MOVING    = 4
uint8 STATE_FAULT     = 5
uint8 STATE_ESTOP     = 6

std_msgs/Header header

uint8  state
string state_name

JointTelemetry[] joints          # always 12, in the canonical order
FootState[]      feet            # always 4: FL, FR, RL, RR

geometry_msgs/Pose  base_pose    # in `odom`
geometry_msgs/Twist base_twist   # in base_link
float64             base_height_m

float64 battery_voltage_v
float64 estimated_power_w        # sum of I^2*R + mechanical over all joints

ControllerState  controller
SafetyStatus     safety
SimulationState  simulation
