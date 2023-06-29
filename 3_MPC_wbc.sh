#! /usr/bin/env bash
cd ocs2_ws/
source devel/setup.bash
sleep 2s
rosrun ocs2_wbc_ros gazebo_sim
