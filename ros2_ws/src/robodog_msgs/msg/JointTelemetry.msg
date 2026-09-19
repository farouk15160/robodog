# Everything known about one joint for one control cycle. `*_command` fields
# report the value actually sent to the actuator, i.e. AFTER safety clamping,
# so the GUI can show the difference between what was asked for and what was
# allowed.

uint16 FAULT_NONE             = 0
uint16 FAULT_POSITION_LIMIT   = 1
uint16 FAULT_VELOCITY_LIMIT   = 2
uint16 FAULT_TORQUE_LIMIT     = 4
uint16 FAULT_OVERTEMPERATURE  = 8
uint16 FAULT_COMMUNICATION    = 16
uint16 FAULT_ENCODER          = 32
uint16 FAULT_UNDERVOLTAGE     = 64
uint16 FAULT_OVERCURRENT      = 128
uint16 FAULT_NOT_ENABLED      = 256
uint16 FAULT_WATCHDOG         = 512

string  name
float64 position           # [rad]    measured, output shaft
float64 velocity           # [rad/s]  measured
float64 effort             # [N.m]    estimated output torque
float64 current            # [A rms]  = effort / torque_constant
float64 temperature        # [degC]   winding; modelled in simulation
float64 position_command   # [rad]    as sent to the actuator
float64 velocity_command   # [rad/s]
float64 effort_command     # [N.m]
float64 kp
float64 kd
float64 torque_utilisation # effort / continuous_torque_limit, 1.0 = at rating
uint8   mode
uint16  fault_flags
bool    enabled
