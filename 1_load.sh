#! /usr/bin/env bash
cd catkin_ws/
source devel/setup.sh
sleep 2s
roslaunch jy_control_test load_X20.launch
