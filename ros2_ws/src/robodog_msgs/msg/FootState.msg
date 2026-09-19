string               name
geometry_msgs/Point  position_in_base   # foot frame origin, base_link coords
geometry_msgs/Vector3 velocity_in_base
bool                 contact
float64              normal_force       # [N] estimated, 0 when not in contact
