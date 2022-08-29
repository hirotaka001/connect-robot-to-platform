#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from dongle_bridge.conv_mqtt2ros import Mqtt2Ros

NODE_NAME = 'conv_mqtt2ros'


def main():
    try:
        # for Start conv_mqtt2ros Node
        rospy.init_node(NODE_NAME)
        Mqtt2Ros().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
