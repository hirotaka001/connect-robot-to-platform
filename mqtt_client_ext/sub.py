#!usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import msgpack
import paho.mqtt.client as mqtt     # MQTTのライブラリをインポート

# ブローカーに接続できたときの処理
def on_connect(client, userdata, flag, rc):
  print("Connected with result code " + str(rc))  # 接続できた旨表示
  client.subscribe(args[1])  # subするトピックを設定

# ブローカーが切断したときの処理
def on_disconnect(client, userdata, rc):
  if  rc != 0:
    print("Unexpected disconnection.")

# メッセージが届いたときの処理
def on_message(client, userdata, msg):
  # msg.topicにトピック名が，msg.payloadに届いたデータ本体が入っている
  print("Received message on topic '" + msg.topic + "' with QoS " + str(msg.qos))
  print(msgpack.unpackb(msg.payload))
  # type(msgpack.unpackb(msg.payload)) = <class 'dict'>

# Main関数
def main(args):
  client = mqtt.Client()                 # クラスのインスタンス(実体)の作成
  client.on_connect = on_connect         # 接続時のコールバック関数を登録
  client.on_disconnect = on_disconnect   # 切断時のコールバックを登録
  client.on_message = on_message         # メッセージ到着時のコールバック
  client.username_pw_set("rbag","rbag")  # アカウント情報

  client.connect("localhost", 1883, 60)  # 接続先は自分自身

  client.loop_forever()                  # 永久ループして待ち続ける

if __name__ == '__main__':          # importされないときだけmain()を呼ぶ
  args = sys.argv
  if len(args) != 2:
    print("Usage: python sub.py <$1 MQTT Topic>")
    exit(0)

  print('Topic: ' + args[1])
  main(args)
