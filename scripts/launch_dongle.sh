#!/bin/sh

# for launch /default/mqtt_bridge
cd /home/pi/ros_catkin_ws/src/connect-robot-to-platform
sudo cp mqtt_bridge_ext/config/mqtt_bridge_default.yaml /opt/ros/melodic/share/mqtt_bridge/config/mqtt_bridge_default.yaml
sudo cp mqtt_bridge_ext/launch/mqtt_bridge.launch /opt/ros/melodic/share/mqtt_bridge/launch/mqtt_bridge.launch
export ROS_PACKAGE_PATH=/home/pi/ros_catkin_ws/src/xacro:/opt/ros/melodic/share
roslaunch mqtt_bridge mqtt_bridge.launch &

# for launch dongle_bridge
cd /home/pi/ros_catkin_ws/src/connect-robot-to-platform
sudo chmod +x dongle_bridge/scripts/conv_mqtt2ros.py
sudo chmod +x dongle_bridge/scripts/mqtt_bridge_manager.py
export ROS_PACKAGE_PATH=/home/pi/ros_catkin_ws/src:/opt/ros/melodic/share
roslaunch dongle_bridge dongle_bridge.launch &
