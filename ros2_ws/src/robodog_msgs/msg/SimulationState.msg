# Present only when a simulation backend is active. `backend` is 'none' on the
# real robot, which is how the GUI decides whether to show the sim panel.
std_msgs/Header header
bool    active
string  backend          # 'mujoco' | 'kinematic' | 'none'
string  world            # world file currently loaded
float64 sim_time_s
float64 wall_time_s
float64 realtime_factor
float64 timestep_s
uint64  steps
bool    paused
