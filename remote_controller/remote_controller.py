#!usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import importlib

# コントローラー入力取得用ゲームライブラリ
import pygame
from pygame.locals import *

import paho.mqtt.client as mqtt     # MQTTのライブラリをインポート
import msgpack


def on_connect(client, userdata, flag, rc):
    """
    ブローカーに接続したときにターミナルにログを出力する

    Parameters
    ----------
    client:     コールバックのためのMQTTクライアントのインスタンス
    userdata:   Client() もしくは userdata_set() にセットされるプライベートなユーザデータ
    flags:      ブローカーから送信される応答フラグ
    rc:         MQTT通信の接続結果
    """

  print("Connected with result code " + str(rc))


def on_disconnect(client, userdata, rc):
    """
    ブローカーが切断したときにターミナルにログを出力する
    （MQTTのライブラリがv3.1系のバージョンで使用できる）

    Parameters
    ----------
    client:     コールバックのためのMQTTクライアントのインスタンス
    userdata:   Client() もしくは userdata_set() にセットされるプライベートなユーザデータ
    rc:         MQTT通信の接続結果
    """

  if rc != 0:
     print("Unexpected disconnection.")


def on_publish(client, userdata, mid):
    """
    パブリッシュが完了したときにターミナルにログを出力する

    Parameters
    ----------
    client:     コールバックのためのMQTTクライアントのインスタンス
    userdata:   Client() もしくは userdata_set() にセットされるプライベートなユーザデータ
    mid:        パブリッシュリクエストのメッセージID（Integer型）
    """

  print("publish: {0}".format(mid))


def mqtt_pub(client, topic, msg):
    """
    MQTTブローカーへパブリッシュが完了したときにターミナルにログを出力する

    Parameters
    ----------
    client:     コールバックのためのMQTTクライアントのインスタンス
    topic:      パブリッシュする宛先トピック
    msg:        パブリッシュするメッセージ（String）
    """

  # ログを標準出力する
  print("Send Msg : " + msg)
  # メッセージをバイト配列に変換する
  pub_msg = bytearray(msgpack.packb({"data": msg}, use_bin_type=False))
  # MQTTブローカーへパブリッシュする
  client.publish(topic, pub_msg)


def main(args):
    """
    ロボットに指定したトピックへコマンドをパブリッシュする
    （importされたときのメイン関数）

    Parameters
    ----------
    args[0]:    実行ファイル名（remote_control.py）
    args[1]:    宛先トピック名
    args[2]:    宛先ロボットID
    """

  # MQTTクライアントのインスタンス(実体)を作成
  client = mqtt.Client()
  # 接続時のコールバック関数を登録
  client.on_connect = on_connect
  # 切断時のコールバックを登録
  client.on_disconnect = on_disconnect
  # メッセージ送信時のコールバック関数を登録
  client.on_publish = on_publish
  # MQTTブローカーのアカウント情報を登録
  client.username_pw_set("rbag","rbag")
  # 自分自身の筐体にあるMQTTブローカーに接続
  client.connect("localhost", 1883, 60)

  # 通信処理スタート
  # subはloop_forever()だが，pubはloop_start()で起動だけさせる
  client.loop_start()

  # 初期化
  # 宛先トピック名
  to_topic = args[1]
  # 宛先ロボットID
  to_robot = args[2]

  # コントローラの有無を確認
  # ジョイスティックモジュールを初期化
  pygame.joystick.init()
  try:
    # ジョイスティックインスタンスを生成
    joystick = pygame.joystick.Joystick(0)
    # ジョイスティックインスタンスを初期化
    joystick.init()
    # ログを標準出力
    # ジョイスティックの名前
    print('JoyStick Name : ', joystick.get_name())
    # ジョイスティックのボタン数
    print('Num of Button : ', joystick.get_numbuttons())
    # ジョイスティックの方向軸の数
    print(' Num of Axis  : ', joystick.get_numaxes())
    # ジョイスティックのハットボタン（十字キー）の数
    print(' Num of Hat   : ', joystick.get_numhats())
  except pygame.error:
    # エラーログを標準出力
    print('The joystick is not connected.')
    # プログラムを終了
    exit(-1)

  # pygameモジュールを初期化
  pygame.init()

  # コントローラを識別
  controller = None
  # PS3コントローラのモジュールをインポート
  if joystick.get_name() == 'Sony PLAYSTATION(R)3 Controller':
    controller = importlib.import_module("controller.ps3")
  # F710ワイヤレスゲームパッドのモジュールをインポート
  elif joystick.get_name() == 'Logicool Cordless RumblePad 2':
    controller = importlib.import_module("controller.f710")
  else:
    # コントローラを認識出来ない場合はエラーログを標準出力
    print('The joystick is not supported.')
    # プログラムを終了
    exit(-1)    
  # 全てのジョイスティックの方向軸の数を取得
  num_axis = joystick.get_numaxes()

  # コントローラー操作の入力を取得する
  # 取得の制御フラグ
  active = True
  # ジョイスティックの傾斜を初期化
  analog_level = 0
  while active:
    # イベントキューからイベントを取得
    for e in pygame.event.get():
      # 終了処理
      if e.type == QUIT:
        active = False

      # コントローラー操作からメッセージを生成
      cmd_str = ""
      # ボタンを押す操作
      if e.type == pygame.locals.JOYBUTTONDOWN:
        # ジョイスティックの傾斜を初期化
        analog_level = 0
        # ボタンを押す操作は無効
        cmd_str = controller.check_button(e.button)
      # ジョイスティックの操作
      elif e.type == pygame.locals.JOYAXISMOTION:
        axes = []
        # 全てのジョイスティックの全方向の入力を取得
        for i in range(num_axis):
          axes.append(joystick.get_axis(i))
        # 制御コマンドとジョイスティックの傾斜を計算
        cmd_str, analog_level = controller.check_axis(axes, analog_level)
      # 十字キーの操作
      elif e.type == pygame.locals.JOYHATMOTION:
        # 十字キーの押されたボタンを取得（f710.pyのみ使用）
        hat = joystick.get_hat(0)
        # 十字キーの押された方向を計算
        cmd_str = controller.check_hat(hat)
      # ボタンを離す操作
      elif e.type == pygame.locals.JOYBUTTONUP:
        # ジョイスティックの傾斜を初期化
        analog_level = 0
        # 動作を停止する
        cmd_str = controller.cmd_stop()

      # コマンドをパブリッシュする
      if cmd_str != "":
        mqtt_pub(client, to_topic, to_robot + cmd_str)


if __name__ == '__main__':
    """
    指定したトピックとロボットにコマンドをパブリッシュする
    （importされないときのメイン関数）

    Parameters
    ----------
    args[0]:    実行ファイル名（remote_control.py）
    args[1]:    宛先MQTTトピック名
    args[2]:    宛先ロボットID
    """

  args = sys.argv
  # 引数の数をチェック
  if len(args) != 3:
    print("Usage: python remote_control.py {$1 MQTT Topic} {$2 Target Robot ID}")
    exit(0)

  # 宛先MQTTトピック名を標準出力
  print("     Send Topic : " + args[1])
  # 宛先ロボットIDを標準出力
  print("Target Robot ID : " + args[2])
  # メイン関数を呼び出し
  main(args)
