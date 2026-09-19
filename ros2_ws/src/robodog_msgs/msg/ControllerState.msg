# What the control stack is doing, independent of the joints themselves.
std_msgs/Header header
string   active_controller   # 'idle' | 'joint' | 'pose' | 'gait' | 'trajectory'
uint8    control_mode        # JointCommand.MODE_*
string   active_gait
string   active_pose
bool     trajectory_active
float64  trajectory_progress # 0 .. 1
float64  update_rate_hz
string[] available_controllers
