# Output of robodog_control/safety_monitor, published every control cycle.
std_msgs/Header header

bool     estop_engaged      # output currently inhibited
bool     estop_latched      # requires an explicit reset even if the cause cleared
string   estop_source       # 'gui', 'service', 'watchdog', 'fault', ...

uint16   active_faults      # bitwise OR of every joint's fault_flags
string[] messages           # human-readable, newest first, bounded length

float64  max_temperature_c
string   hottest_joint
float64  max_torque_utilisation
string   most_loaded_joint

uint32   clamp_events       # cumulative count of commands modified by the limiter
bool     watchdog_ok
float64  command_age_s      # time since the last accepted command
float64  loop_period_s      # measured control period
float64  loop_jitter_s      # max |measured - nominal| over the last window
