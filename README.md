WBC的输入为世界坐标系下的角速度；
MPC的输入为世界坐标系下的状态量；
Control Frame: roll、pitch为世界坐标系下的状态量，yaw与机器人朝向随动；
WBC中RBDL为四元数表达；


## In branch x20CloseLoop

### in catkin_ws:

    source devel/setup.bash
    roslaunch jy_control_test load_X20.launch
    rosrun jy_control_test wbc_pd_trot_mpc
    roslaunch pronto_x20 pronto_x20.launch

### in ocs2_ws:

    source devel/setup.bash
    roslaunch ocs2_jypro legged_robot.launch
    rosrun ocs2_jypro MPCProntoConversion
