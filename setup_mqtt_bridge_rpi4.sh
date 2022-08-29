#!/bin/sh

# Install mosquitto
sudo apt-get install -y mosquitto mosquitto-clients

# Git clone mqtt_bridge
cd ~/ros_catkin_ws/src
git clone https://github.com/groove-x/mqtt_bridge

# Build mqtt_bridge package
cd ~/ros_catkin_ws
sudo ./src/catkin/bin/catkin_make_isolated --pkg mqtt_bridge --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j4
# catkin_make --pkg mqtt_bridge
# rosdep install mqtt_bridge

