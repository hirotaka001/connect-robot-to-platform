#!/bin/sh

# for launch /robot_bridge/mqtt_bridge
sudo cp /home/pi/ros_catkin_ws/src/connect-robot-to-platform/mqtt_bridge_ext/config/mqtt_bridge.yaml /opt/ros/melodic/share/mqtt_bridge/config/mqtt_bridge_robot.yaml
sudo cp /home/pi/ros_catkin_ws/src/connect-robot-to-platform/mqtt_bridge_ext/launch/mqtt_bridge.launch /opt/ros/melodic/share/mqtt_bridge/launch/mqtt_bridge.launch
export ROS_PACKAGE_PATH=/home/pi/ros_catkin_ws/src/xacro:/opt/ros/melodic/share
roslaunch mqtt_bridge mqtt_bridge.launch mode:=robot &
