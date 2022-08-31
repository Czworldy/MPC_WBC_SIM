WBC的输入为世界坐标系下的角速度；
MPC的输入为世界坐标系下的状态量；
Control Frame: roll、pitch为世界坐标系下的状态量，yaw与机器人朝向随动；
WBC中RBDL为四元数表达；

sudo pwd: '

1. ~/MPC_WBC_sim/catkin_ws$ roslaunch jy_control_test load.launch //gazebo run
2. ~/MPC_WBC_sim/catkin_ws$ rosrun jy_control_test wbc_pd_trot_mpc // wait for standing in gazebo
3. ~/MPC_WBC_sim/catkin_ws$ roslaunch pronto_jypro pronto_jypro.launch // wait for est
source devel/setup.bash

4. ocs2_ws roslaunch ocs2_jypro legged_robot.launch // MPC is reset -> 4 windows -> 1. gait (standing_trot)(trot) 2. ref (x y yaw)
(x y z(m) yaw(deg))

5. ocs2_ws rosrun ocs2_jypro MPCProntoConversion 
