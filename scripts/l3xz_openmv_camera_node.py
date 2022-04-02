#!/usr/bin/python3
#-*- coding: utf-8 -*-

import rospy
import roslib
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo 

import cv2
import numpy as np
from camera_interface import OpenMvInterface

import time

def main():
  rospy.init_node('openmv')

  frame_id = rospy.get_param("~frame_id", "odom")

  show = rospy.get_param("~show_image", False)

  rate = rospy.Rate(rospy.get_param("~rate_hz", 10))

  omv_cam = OpenMvInterface(rospy.get_param("~port", "/dev/ttyACM0"),
     rospy.get_param("~camera_script", "/home/pi/catkin_ws/src/l3xz_openmv_camera/src/interfacecamera_script.py"))
 
  img_name = rospy.get_param("~image_topic", "image_color")
  pub_image = rospy.Publisher(img_name,
          Image, queue_size = rospy.get_param("~image_queue", 1))
  pub_info = rospy.Publisher(rospy.get_param("~info_topic", "camera_info"),
          CameraInfo, queue_size = rospy.get_param("~image_queue", 1))

  img_msg = Image()
  img_msg.header.frame_id = frame_id

  info_msg = CameraInfo()
  info_msg.header.frame_id = frame_id

  while not rospy.is_shutdown():
    
    framedump = omv_cam.dump()
    if framedump is not None:
      if show:
        cv2.imshow(img_name, framedump.pixels)
        cv2.waitKey(1)
      time = rospy.Time.now()
      img_msg.header.stamp = time 
      pub_image.publish(framedump.fill_img_msg(img_msg))

      info_msg.header.stamp = time
      pub_info.publish(framedump.fill_info_msg(info_msg))

    rate.sleep()

if __name__ == '__main__':
  main()
