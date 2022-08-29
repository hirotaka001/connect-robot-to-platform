#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from dongle_bridge.mqtt_bridge_manager import MqttBridgeManager

NODE_NAME = 'mqtt_bridge_manager'


def main():
    try:
        # ノードを初期化する
        rospy.init_node(NODE_NAME)

        # ノードが終了するまで待機する
        MqttBridgeManager().start()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
