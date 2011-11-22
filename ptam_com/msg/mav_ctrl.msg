Header header

int8        type        #message type

# control commands, all units in SI units !!!
# There are 3 operating modes of the helicopter:
# 1. Acceleration: x, y, z correspond to x_dotdot etc... with the exception that yaw is an angular velocity
#    Inputs must be in body-coordinates
#    Currently x~pitch, y~roll, z~thrust, units in rad and rad/s for yaw
#    In future real accelerations in m/s^2 and yaw in rad/s
# 2. Velocity: x, y, z, yaw correspond to x_dotdot etc...
#    Vehicle must have it's own pose estimation e.g. vision/GPS
#    Inputs must be in body-coordinates
#    Units in m/s and rad/s respectively
# 3. Position
#    Vehicle must have it's own pose estimation e.g. vision/GPS
#    Inputs in fixed coordinate system

float32     x
float32     y
float32     z
float32     yaw

int8 acceleration = 1
int8 velocity = 2
int8 position = 3

# old message
# all control commands in heli-coordinates!!!
#float32     roll    # roll angle in rad                 ~ acceleration in x direction of heli  
#float32     pitch   # pitch angle in rad                ~ acceleration in y direction of heli
#float32     yaw     # angular velocity yaw in rad/s     angular velocity around z-axis
#float32     thrust  # thrust from 0-100%                ~ acceleration in z direction of heli