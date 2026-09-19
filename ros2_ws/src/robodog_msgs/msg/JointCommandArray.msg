# Command for a set of joints. `names` and `commands` must be the same length.
# Joints omitted from `names` keep their previous command.
std_msgs/Header header
string[]        names
JointCommand[]  commands
