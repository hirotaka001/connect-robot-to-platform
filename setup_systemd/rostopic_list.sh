#!/bin/bash

/bin/bash /opt/dongle-module/bin/.create_rosenv_dongle
source /home/pi/.set_ros_dongle
source /opt/ros/melodic/setup.bash
source /home/pi/ros_catkin_ws/devel_isolated/setup.bash

/opt/ros/melodic/bin/rostopic list

