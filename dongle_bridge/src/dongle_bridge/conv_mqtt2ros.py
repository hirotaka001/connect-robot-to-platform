# -*- coding: utf-8 -*-
import os
import threading
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Mqtt2Ros(object):
    """
    ロボットPFから受信したコマンドをROSメッセージに変換してロボットへ送信する

    Attributes
    ----------
    __turtlebot3_cmd_pub : rospy.Publisher()
        TurtleBot3へパブリッシュする
    __jetbot_cmd_pub : rospy.Publisher()
        JetBotの/jetbot_motors/cmd_strトピックへパブリッシュする（十字キー操作）
    __jetbot_raw_pub : rospy.Publisher()
        JetBotの/jetbot_motors/cmd_rawトピックへパブリッシュする（アナログスティック操作）
    __current_odometry : nav_msgs/Odometry
        現在のロボットのオドメトリ情報
    __is_moving : bool
        ロボットの動作情報
    __lock : threading.Lock()
        プリミティブロックのオブジェクト
    """

    def __init__(self):
        """
        メッセージ送受信の設定をするコンストラクタ
        """

        # ROSのパラメータをOSのパラメータに上書きする
        self.__params = self._override_params(rospy.get_param('~'))

        # /cmd_to_robotトピックで受信したロボットPFのメッセージを購読する
        rospy.Subscriber(self.__params['topics']['cmd_sub'],
                         String,
                         self._on_command_receive,
                         queue_size=10)

        # /cmd_velトピックでTurtleBot3へコマンドを発行する
        self.__turtlebot3_cmd_pub = rospy.Publisher(self.__params['topics']['tb3_cmd_pub'],
                                                    Twist,
                                                    queue_size=10)

        # /odomトピックでTurtleBot3のオドメトリ情報を購読する
        rospy.Subscriber(self.__params['topics']['tb3_odom_sub'],
                         Odometry,
                         self._on_odom_receive,
                         queue_size=10)

        # /jetbot_motors/cmd_strトピックでJetbotへコマンドを発行する
        self.__jetbot_cmd_pub = rospy.Publisher(self.__params['topics']['jb_str_pub'],
                                                    String,
                                                    queue_size=10)

        # /jetbot_motors/cmd_rawトピックでJetbotへコマンドを発行する
        self.__jetbot_raw_pub = rospy.Publisher(self.__params['topics']['jb_raw_pub'],
                                                    String,
                                                    queue_size=10)

        # 現在のロボットのオドメトリ情報を初期化する
        self.__current_odometry = None

        # ロボットの動作情報を初期化する
        self.__is_moving = False

        # プリミティブロックオブジェクトを生成する
        self.__lock = threading.Lock()

    def _override_params(self, params):
        """
        rosparamに設定された値をOSのROS関連の環境変数に変更する

        Parameters
        ----------
        params : XmlRpcLegalValue
            rosparamに設定された値

        Returns
        -------
        params : XmlRpcLegalValue
            OSの環境変数に変更されたrosparam
        """

        # 前後方向の速度を指定する
        if 'POLYGON_VELOCITIES_X' in os.environ:
            try:
                # OSの環境変数を取得する
                pvx = float(os.environ['POLYGON_VELOCITIES_X'])
                # OSの環境変数をrosparamに上書きする
                params['robot']['polygon']['velocities']['x'] = pvx
            # 誤った型の引数が渡された場合はエラーで返す
            except (ValueError, TypeError):
                pass

        # 回転速度を指定する
        if 'POLYGON_VELOCITIES_Z' in os.environ:
            try:
                # OSの環境変数を取得する
                pvz = float(os.environ['POLYGON_VELOCITIES_Z'])
                # OSの環境変数をrosparamに上書きする
                params['robot']['polygon']['velocities']['z'] = pvz
            # 誤った型の引数が渡された場合はエラーで返す
            except (ValueError, TypeError):
                pass

        # 移動距離を指定する
        if 'POLYGON_EDGE_LENGTH' in os.environ:
            try:
                # OSの環境変数を取得する
                pel = float(os.environ['POLYGON_EDGE_LENGTH'])
                # OSの環境変数をrosparamに上書きする
                params['robot']['polygon']['edge']['length_meter'] = pel
            # 誤った型の引数が渡された場合はエラーで返す
            except (ValueError, TypeError):
                pass
        
        # rosparamを返す
        return params

    def start(self):
        """
        ノードが終了するまで待機する
        """
        rospy.spin()

    def _on_command_receive(self, msg):
        """
        ロボットPFから/cmd_to_robotトピックで受信したコマンドを各ロボットに振り分ける

        Parameters
        ----------
        msg : std_msgs.msg
            ロボットPFから受信したコマンド
        """

        # コマンドをROSのログに出力する
        raw_msg = msg.data
        rospy.loginfo("[Received] topic=/cmd_to_robot   msg=" + raw_msg)

        # ロボットIDを見極める
        split_msg = raw_msg.split('|')

        # TurtleBot3にコマンドを送信する
        if split_msg[0] == 'turtlebot3':
            self._on_command_receive_tb3(split_msg[1], split_msg[2])

        # Jetbotにコマンドを送信する
        if split_msg[0] == 'jetbot':
            self._on_command_receive_jb(split_msg[1], split_msg[2])

    def _on_command_receive_tb3(self, cmd, args):
        """
        ロボットPFからのコントローラ操作のコマンドを受けてTurtleBot3を動作させる

        Parameters
        ----------
        cmd : String
            ロボットPFからの動作指示（コマンド）
        args : String
            動作の方向,速度
        """

        # 「動作の方向,速度」を分離する
        split_args = args.split(',')
        # コマンドに対応する動作を決定する
        if cmd == 'move':
            # 前進
            if split_args[0] == 'forward':
                self._move(self._do_forward)
            # 後退
            elif split_args[0] == 'backward':
                self._move(self._do_backward)
            # 左旋回
            elif split_args[0] == 'left':
                self._move(self._do_left)
            # 右旋回
            elif split_args[0] == 'right':
                self._move(self._do_right)
            # 停止
            elif split_args[0] == 'stop':
                self._move(self._do_stop)

    def _on_command_receive_jb(self, cmd, args):
        """
        ロボットPFからコントローラ操作のコマンドを受けてJetBotを動作させる

        Parameters
        ----------
        cmd : String
            ロボットPFからの動作指示（コマンド）
        args : String
            動作の方向,速度
        """

        # 「動作の方向,速度」を分離する
        split_args = args.split(',')
        # コマンドに対応する動作を決定する
        if cmd == 'move':
            # 十字キーの操作
            if len(split_args) == 1:
                self._cross_key_input_jb(split_args[0])
            # アナログスティックの操作
            else:
                # 動作の方向を取得する
                direction = split_args[0]
                cmd_raw = ""
                # 前進
                if direction == 'forward':
                    cmd_raw = split_args[1] + ',' + split_args[1]
                # 後退
                elif direction == 'backward':
                    cmd_raw = str(float(split_args[1]) * -1) + ',' + str(float(split_args[1]) * -1)
                # 左旋回
                elif direction == 'left':
                    cmd_raw = str(float(split_args[1]) * -1) + ',' + split_args[1]
                # 右旋回
                elif direction == 'right':
                    cmd_raw = split_args[1] + ',' + str(float(split_args[1]) * -1)
                # 停止
                elif direction == 'stop':
                    cmd_raw = split_args[1] + ',' + split_args[1]  
                # JetBotへ動作をパブリッシュする             
                self._analog_input_jb(cmd_raw)

    def _analog_input_jb(self, cmd_raw):
        """
        アナログスティックの操作をJetBotにパブリッシュする

        Parameters
        ----------
        cmd_raw : String
            ロボットPFからの動作指示「動作の方向,速度」
        """

        # パブリッシュするトピックのメッセージタイプに変換する
        snd_msg = String()
        snd_msg.data = str(cmd_raw)
        # JetBotに動作指示をパブリッシュする
        self.__jetbot_raw_pub.publish(snd_msg)

    def _cross_key_input_jb(self, cmd_str):
        """
        十字キーの操作をJetBotにパブリッシュする

        Parameters
        ----------
        cmd_str : String
            ロボットPFからの動作指示「動作の方向」
        """

        # パブリッシュするトピックのメッセージタイプに変換する
        snd_msg = String()
        snd_msg.data = str(cmd_str)
        # JetBotに動作指示をパブリッシュする
        self.__jetbot_cmd_pub.publish(snd_msg)

    def _on_odom_receive(self, msg):
        """
        TurtleBot3のオドメトリ情報を設定する

        Parameters
        ----------
        msg : nav_msgs.msg
            オドメトリ情報
        """

        # 現在のオドメトリ情報を設定する
        self.__current_odometry = msg

    def _do_forward(self):
        """
        TurtleBot3の前進動作を実行する
        """

        self._forward(self.__params['robot']['polygon']['velocities']['x'])

    def _do_backward(self):
        """
        TurtleBot3の後退動作を実行する
        """

        self._forward(self.__params['robot']['polygon']['velocities']['x'], reverse=True)

    def _do_left(self):
        """
        TurtleBot3の左旋回動作を実行する
        """

        self._rotate(self.__params['robot']['polygon']['velocities']['z'])

    def _do_right(self):
        """
        TurtleBot3の右旋回動作を実行する
        """

        self._rotate(self.__params['robot']['polygon']['velocities']['z'], reverse=True)

    def _do_stop(self):
        """
        TurtleBot3の停止動作を実行する
        """

        self._forward()

    def _forward(self, linear=0.0, reverse=False):
        """
        TurtleBot3へ前進動作をパブリッシュする

        Parameters
        ----------
        linear : float
            移動の速度
        reverse : bool
            True：後退
            False：前進
        """

        # パブリッシュするトピックのメッセージタイプに変換する
        twist = Twist()
        if reverse:
            # 後退する場合に速度の正負を反転する
            linear *= -1
        # 移動の速度を指定する
        twist.linear.x = linear

        # TurtleBot3に動作指示をパブリッシュする
        self.__turtlebot3_cmd_pub.publish(twist)

    def _rotate(self, angular=0.0, reverse=False):
        """
        TurtleBot3へ旋回動作をパブリッシュする

        Parameters
        ----------
        angular : float
            旋回の速度
        reverse : bool
            True：右旋回
            False：左旋回
        """

        # パブリッシュするトピックのメッセージタイプに変換する
        twist = Twist()
        if reverse:
            # 左旋回する場合に速度の正負を反転する
            angular *= -1
        # 旋回の速度を指定する
        twist.angular.z = angular

        # TurtleBot3に動作指示をパブリッシュする
        self.__turtlebot3_cmd_pub.publish(twist)

    def _move(self, callback):
        """
        TurtleBot3でコマンドを実行する

        Parameters
        ----------
        callback : コールバック関数
            TurtleBot3の動作
        """

        # TurtleBot3の動作を実行する
        def func():
            # 動作中はコマンドを実行しない
            if self.__is_moving:
                return

            # 動作のステータスを排他処理する
            with self.__lock:
                # 動作が開始する
                self.__is_moving = True

            # 動作を実行する
            callback()

            # 動作のステータスを排他処理する
            with self.__lock:
                # 動作が終了する
                self.__is_moving = False
        # funcメソッドのスレッドを生成する
        t = threading.Thread(target=func)
        # スレッドを実行する
        t.start()
        # スレッドを返す
        return t
