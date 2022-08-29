# -*- coding: utf-8 -*-
import time
import subprocess

import rospy
import rosnode
from std_msgs.msg import String


class MqttBridgeManager(object):
    """
    mqtt_bridgeの管理とトピックの設定を行う

    Attributes
    ----------
    __return_to_rpf : rospy.Publisher()
        ロボットPFへパブリッシュする
    __bridge_topics : list
        MQTT BridgeからロボットPFへ通知する際に除外するトピック
    """

    def __init__(self):
        """
        メッセージ送受信の設定をするコンストラクタ
        """

        # __paramsをrosparamで初期化する
        self.__params = rospy.get_param('~')

        # /cmd_to_dongleトピックで受信したロボットPFのメッセージを購読する
        rospy.Subscriber(self.__params['topics']['cmd_to_dongle'],
                         String,
                         self._on_command_receive,
                         queue_size=10)

        # /return_to_rpfトピックでドングルのメッセージをロボットPFへ発行する
        self.__return_to_rpf = rospy.Publisher(self.__params['topics']['return_to_rpf'],
                                                    String,
                                                    queue_size=10)

        # ロボットPF側で監視不要なトピックであるため、MQTT BridgeからロボットPFへ通知する際に除外するトピック名
        # /cmd_to_robot：ロボットPFからロボット向けにコマンドを送信するトピック
        # /cmd_to_dongle：ロボットPFからドングル向けにコマンドを送信するトピック
        # /return_to_rpf：ドングルからロボットPF向けにコマンドを送信するトピック
        self.__bridge_topics = [self.__params['topics']['cmd_to_robot'], self.__params['topics']['cmd_to_dongle'], self.__params['topics']['return_to_rpf']]

    def start(self):
        """
        ノードが終了するまで待機する
        """
        rospy.spin()

    def _on_command_receive(self, msg):
        """
        ロボットPFから受信したメッセージからトピックを操作するコマンドを決定する

        Parameters
        ----------
        msg : std_msgs.msg(String)
            ロボットPFから受信したメッセージ
            RobotID|Command|Args|
        """

        # メッセージを取得する
        raw_command = msg.data
        # メッセージをログに出力する
        rospy.loginfo("[Received] topic=/cmd_to_dongle   msg=" + raw_command)
        # メッセージを"|"で分割する
        split_command = raw_command.split('|')

        # コマンドに対応する動作を決定する
        if split_command[1] == 'rostopic list':
            # 接続ロボットのROSトピック一覧を取得する
            self._rostopic_list()
        elif split_command[1] == 'bridge':
            # ブリッジするトピックをPFから設定する
            self._bridge(raw_command, split_command[2])

    def _rostopic_list(self):
        """
        ロボットのROSトピック一覧を取得してロボットPFにパブリッシュする
        """

        # パブリッシュしている全トピック("/")の一覧を取得する
        topics = rospy.get_published_topics('/')

        # トピック一覧をString型に書き換えてパブリッシュする
        msg = "Topic,MsgType|"
        for t in topics:
            # ロボットPFにパブリッシュしないトピック名を除く
            if t[0] not in self.__bridge_topics:
                # 各トピックを連結する t[0]:トピック名、t[1]:メッセージタイプ
                msg = msg + t[0] + "," + t[1] + "|"
        # ロボットPFへトピック一覧をパブリッシュする
        self.__return_to_rpf.publish(msg)

    def _bridge(self, msg, target_topics):
        """
        ロボットで収集したいトピックをロボットPFから設定する

        Parameters
        ----------
        msg : String
            ロボットPFから受信したメッセージ
            RobotID|Command|Args|
        target_topics : String
            ロボットPFから受信したメッセージ
            Args
        """

        # 既に起動しているMQTT Bridgeを停止する
        rosnode.kill_nodes(['robot/mqtt_bridge'])

        # MQTT Bridge用のconfigを生成するスクリプトを実行し、実行結果をROSのlogに出力する
        rospy.loginfo(subprocess.check_output(['/usr/bin/python', '/home/pi/ros_catkin_ws/src/connect-robot-to-platform/mqtt_bridge_ext/config/generate_config.py', target_topics]))

        # MQTT Bridgeを起動するlaunchファイルを開く
        proc = subprocess.Popen(['/bin/sh', '/home/pi/ros_catkin_ws/src/connect-robot-to-platform/scripts/launch_mqtt_bridge.sh'])

        # MQTT Bridgeの起動を待つ
        time.sleep(3)
        # トピックの設定が完了したことをロボットPFへパブリッシュする
        self.__return_to_rpf.publish('Completed. (Msg = ' + msg + ')')
