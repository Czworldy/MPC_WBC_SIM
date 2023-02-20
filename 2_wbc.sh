#! /usr/bin/env bash
cd catkin_ws/
source devel/setup.sh
sleep 2s
rosrun jy_control_test wbc_pd_trot_mpc
