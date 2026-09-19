# Request for the gait generator. Parameters not set by the requester keep the
# value from robodog_control/config/gaits.yaml for the selected gait.
string  gait              # 'idle','stand','sit','trot','walk','pace','bound','joint_test'
bool    enable
float64 step_frequency_hz
float64 step_height_m
float64 stance_height_m
float64 duty_factor       # fraction of the cycle a foot is in stance
geometry_msgs/Twist velocity   # desired base velocity in base_link
