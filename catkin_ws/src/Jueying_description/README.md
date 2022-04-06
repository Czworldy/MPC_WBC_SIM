# Jueying_description (XACRO)
## Overview

This package contains a simplified robot description (XACRO) of the [Jueying robot](http://www.deeprobotics.cn/) developed by DeepRobotics.

**Author & Maintainer: [DeepRobotics](http://www.deeprobotics.cn/)**

[![Jueying_description](img/Jueying_gazebo.png)](img/Jueying_gazebo.png)

## License

This software is released under a [MIT license](LICENSE).

## Usage

Load the Jueying description in Gazebo:

    roslaunch jueying_gazebo jueying_world.launch

jueying_description: <br>
xacro files of Jueying, as well as some configuration files for Gazebo <br>
jueying_control: <br>
readin.cpp for getting the data of IMU,Xbox360 Joystick,joint velocity/effort/position <br>
motion.cpp is a simple example of PD control method to drive 12 joints

### dependencies

Ubuntu16.04, ROS kinetic, Gazebo7.0
