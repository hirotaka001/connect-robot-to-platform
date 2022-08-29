#!usr/bin/env python
# -*- coding: utf-8 -*-
import sys

# コントローラー入力取得用
import pygame
from pygame.locals import *
import math

# AXIS Idx List
ANALOG_X = 0
ANALOG_Y = 1

# Cross Key
HAT_X = 0
HAT_Y = 1

# Command List
MOVE = "move"
FORWARD = "forward"
BACKWARD = "backward"
LEFT = "left"
RIGHT = "right"
STOP = "stop"

# スティック入力の処理
def check_axis(axes, analog_level):
  cmd_str = ""

  # アナログスティックによる操作
  cmd_str, analog_level = check_axis_analog(axes[ANALOG_X], -1 * axes[ANALOG_Y], analog_level)

  return cmd_str, analog_level

# スティック入力処理
def check_axis_analog(x, y, analog_level):
  cmd_str = ""

  # 領域の算出
  r = math.atan2(y, x)
  if r < 0.0:
    r = r + 2 * math.pi
  degree = r * 360 / (2 * math.pi)

  # 移動方向の算出
  tmp_level = analog_level
  direction = "none"
  if 45.0 <= degree < 135.0:
    tmp_level = int(y / 0.2)
    direction = FORWARD
  elif 135.0 <= degree < 225.0:
    tmp_level = int(-1 * x / 0.2)
    direction = LEFT
  elif 225.0 <= degree < 315.0:
    tmp_level = int(-1 * y / 0.2)
    direction = BACKWARD
  else:
    tmp_level = int(x / 0.2)
    direction = RIGHT

  # 移動指示の送信
  if analog_level != tmp_level:
    if tmp_level == 0:
      cmd_str = "|" + MOVE + "|" + STOP + ",0.0|"
    else:
      cmd_str = "|" + MOVE + "|" + direction + "," +  str(tmp_level * 0.2) + "|"

  return cmd_str, tmp_level

# ハットスイッチの処理
def check_hat(hat):
  cmd_str = ""
  x = hat[HAT_X]
  y = hat[HAT_Y]

  if y > 0: # Up key
    cmd_str = "|" + MOVE + "|" + FORWARD + "|"
  elif y < 0: # Down key
    cmd_str = "|" + MOVE + "|" + BACKWARD + "|"
  elif x < 0: # Left key
    cmd_str = "|" + MOVE + "|" + LEFT + "|"
  elif x > 0: # Right key
    cmd_str = "|" + MOVE + "|" + RIGHT + "|"

  return cmd_str

# ボタンを押す操作の処理
def check_button(button):
  # ボタンを押す操作は無効
  cmd_str = ""
  return cmd_str
"""
  if button == UP_KEY: # Up key
    cmd_str = "|" + MOVE + "|" + FORWARD + "|"
  if button == DOWN_KEY: # Down key
    cmd_str = "|" + MOVE + "|" + BACKWARD + "|"
  if button == LEFT_KEY: # Left key
    cmd_str = "|" + MOVE + "|" + LEFT + "|"
  if button == RIGHT_KEY: # Right key
    cmd_str = "|" + MOVE + "|" + RIGHT + "|"
"""

# 停止処理
def cmd_stop():
  return "|" + MOVE + "|" + STOP + "|"
