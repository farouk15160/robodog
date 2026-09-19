# A single-joint command. This is deliberately the ROBSTRIDE02 native
# motion-control frame: the motor closes
#     tau = kp * (position - q) + kd * (velocity - qd) + effort
# in firmware at its own rate. Sending anything narrower (position only) would
# force the host to close a loop the actuator already closes better.
#
# Set kp = kd = 0 and mode = MODE_TORQUE for pure feed-forward torque control.
# Set effort = 0 with kp, kd > 0 for pure impedance control.

uint8 MODE_IDLE      = 0   # no output, motor free
uint8 MODE_POSITION  = 1   # kp/kd from the controller defaults
uint8 MODE_VELOCITY  = 2
uint8 MODE_TORQUE    = 3   # feed-forward only
uint8 MODE_IMPEDANCE = 4   # full kp/kd/effort triple, used by the gait layer

uint8   mode
float64 position    # [rad]    target, canonical joint convention
float64 velocity    # [rad/s]  target
float64 effort      # [N.m]    feed-forward torque, output-referred
float64 kp          # [N.m/rad]   0 .. 500
float64 kd          # [N.m.s/rad] 0 .. 5
