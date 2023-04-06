# MPC Terrain Msg
geometry_msgs/Quaternion  quaternion # head aligned with robot's heading-axis, for X20 it's X axis.
# These parameters are used to determine the terrain equation
# ax + by + z + d = 0
float32    a
float32    b
float32    d

float32[4] feetHeight # [lf lh rf rh]    
bool[4] contact # [lf lh rf rh]    