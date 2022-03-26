#!/usr/bin/python3
#-*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Image

import cv2
import numpy as np

from camera_interface import OpenmvDumper

import time

d = OpenmvDumper()
rospy.init_node('openmv')
pub = rospy.Publisher("img", Image, queue_size = 1)
while True:
  img = d.dump()
  r = rospy.Rate(100)
  if img is not None:
    cv2.imshow("img", img)
    cv2.waitKey(1)
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.encoding = "bgr8"
    msg.step = 3 * 320
    msg.width = 320
    msg.height = 240
    msg.data = img.tostring()
    print("publish")
    pub.publish(msg)

  r.sleep()
