#!/bin/sh

rosnode kill conv_mqtt2ros
rosnode kill mqtt_bridge_manager
rosnode kill default/mqtt_bridge
rosnode kill robot/mqtt_bridge
