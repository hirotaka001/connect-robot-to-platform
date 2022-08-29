#!/bin/bash

source /home/pi/.set_ros_dongle
source /opt/ros/melodic/setup.bash
source /home/pi/ros_catkin_ws/devel_isolated/setup.bash

export ROS_PACKAGE_PATH=/home/pi/ros_catkin_ws/src/xacro:/opt/ros/melodic/share
/opt/ros/melodic/bin/roslaunch mqtt_bridge mqtt_bridge.launch &

export ROS_PACKAGE_PATH=/home/pi/ros_catkin_ws/src:/opt/ros/melodic/share
/opt/ros/melodic/bin/roslaunch dongle_bridge dongle_bridge.launch &

