#!/usr/bin/python3
#-*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image

import cv2

from camera_interface import OpenmvDumper

import time

d = OpenmvDumper()

while True:
  d.dump()
  time.sleep(1)
