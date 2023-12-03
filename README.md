WBC的输入为世界坐标系下的角速度；
MPC的输入为世界坐标系下的状态量；
Control Frame: roll、pitch为世界坐标系下的状态量，yaw与机器人朝向随动；
WBC中RBDL为四元数表达；

# 2023/06/30
现在这个包提供了三个仿真模式\
    1.Gazebo + Topic 这也就是最常用的gazebo \
    2.Gazebo + ros_controller 之前试过但是由于轨迹由于不明原因太慢而废弃\
    3.raisim
 ---   
 ## gazebo 的仿真可能站起来偏低，只需要rosrun ocs2_jypro legged_robot_target敲一下回车就会升高 这个是没给参考地面估计没运行
 ---
 ## 我强烈怀疑之前走的慢和use_sim_time有关，确保下发轨迹的时间对的上 建议再试试Gazebo + ros_controller，因为这个的控制就是比Gazebo + Topic好。
 ---
 ## 这三个我都测过了 只需要注意对应的task.info文件，并注意落足约束有没有开启。

# 1.Gazebo + Topic
    1.cd catkin_ws/
    source devel/setup.sh
    roslaunch jy_control_test load_X20.launch #打开gazebo 加载world

    2.cd ocs2_ws/
    source devel/setup.sh
    roslaunch ocs2_jypro legged_robot_param.launch #加载各种配置文件的目录
    #特别注意其中的taskFile这个参数，这个参数指代MPC加载 task.info 文件的位置
    #task.info决定落足约束是否开起 他的2-4行
    useFeetPlacementConstraint      true //多边形约束
    useIKresult                     true //轨迹tracking IK部分
    useEndEffectorTrackingCost      true //轨迹tracking pos, vel部分

    3.cd ocs2_ws/
    source devel/setup.sh
    rosrun ocs2_wbc_ros gazebo_sim #开启MPC WBC 

    --等待站立--

    4.cd catkin_ws/
    source devel/setup.sh
    roslaunch pronto_x20 pronto_x20.launch #用于提供接触状态

    5.cd ocs2_ws/roslaunch legged_controllers load_controller.launch
    source devel/setup.sh
    rosrun ocs2_jypro legged_robot_target_traj #开启轨迹接口

# 2.raisim
    1.cd ocs2_ws
    roslaunch ocs2_jypro legged_robot_param.launch
    # 这里输入步态

    2.cd ocs2_ws
    rosrun ocs2_wbc_ros mpc_sim

    3. cd ocs2_ws/
    source devel/setup.sh
    rosrun ocs2_jypro legged_robot_target_traj #开启轨迹接口

## 2.1 raisim RL
    1.cd ocs2_ws
    roslaunch ocs2_jypro legged_robot_raisim.launch
    #等于上面raisim的三步 外加自动standing_trot2（通过载入task_auto_trot.info实现）

# 3.Gazebo + ros_controller
    cd ocs2_ws
    roslaunch legged_x20_description empty_world.launch #这里你可以改world

    cd ocs2_ws
    roslaunch legged_controllers load_controller.launch cheater:=true #这个文件基本等于 legged_robot_param.launch 里面也有taskFile等参数
    #这个窗口可以继续输入步态

    3.rqt 打开plugins->robot tools->controller manager
    找到legged_cheater_controller右键start

    4.cd ocs2_ws/
    source devel/setup.sh
    rosrun ocs2_jypro legged_robot_target_traj #开启轨迹接口

    





