#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import os
import shutil
import sys

def main(args):
    # テンプレートからconfigファイルを作成
    config = "/home/pi/ros_catkin_ws/src/connect-robot-to-platform/mqtt_bridge_ext/config/mqtt_bridge.yaml"
    if os.path.exists("/home/pi/ros_catkin_ws/src/connect-robot-to-platform/mqtt_bridge_ext/config/template_mqtt_bridge.yaml"):
        shutil.copy("/home/pi/ros_catkin_ws/src/connect-robot-to-platform/mqtt_bridge_ext/config/template_mqtt_bridge.yaml", config)

    # ブリッジするTopicを指定
    bridge_topics = []
    if len(args) > 1:
        bridge_topics = args[1].split(',')

    # トピック情報の取得
    topics = rospy.get_published_topics('/')

    # ロボット側のトピック情報をconfigに書き出し
    with open(config, mode='a') as f:
        for t in topics:
            if t[0] in bridge_topics or len(bridge_topics) == 0:
                f.write("  - factory: mqtt_bridge.bridge:RosToMqttBridge\n")
                f.write("    msg_type: " + str(t[1].replace("/",".msg:")) + "\n")
                f.write("    topic_from: " + str(t[0]) + "\n")
                f.write("    topic_to: " + str(t[0][1:]) + "\n")


if __name__ == '__main__':
    args = sys.argv
    main(args)
