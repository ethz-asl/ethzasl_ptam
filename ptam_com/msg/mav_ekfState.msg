Header header
geometry_msgs/Quaternion    orientation         # orientation, necessary for the prediction part
float64[]                   state               # state vector
uint32                      flag                # flag indicating what state currently is

uint32                      currentState = 1
uint32                      initialization = 2
uint32                      statecorrection = 3