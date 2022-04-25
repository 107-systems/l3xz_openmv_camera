#!/usr/bin/python3
#-*- coding: utf-8 -*-
'''
This software is distributed under the terms of the MIT License.
Copyright (c) 2022 107-Systems
Author: Jonas Wühr
'''

import threading
import cv2
import numpy as np
import time

import os
if 'ROS_NAMESPACE' not in os.environ:
  os.environ['ROS_NAMESPACE'] = 'l3xz'

import rospy
import roslib
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo 
from std_msgs.msg import Bool

from l3xz_openmv_camera.srv import rgb, rgbResponse, ir, irResponse, gpio, gpioResponse, gpio_set, gpio_setResponse
from camera_interface import OpenMvInterface

omv_cam = None
pub_inputs = []
mutex = threading.Lock() # Mutex for asynchronous services and publishing.

def rgb_callback(request): # rgb service implementation
  global omv_cam
  global mutex
  mutex.acquire()
  success = omv_cam.rgb_led(request.r, request.g, request.b)
  mutex.release()
  return rgbResponse(success)

def ir_callback(request): # ir service implementation
  global omv_cam
  global mutex
  mutex.acquire()
  success = omv_cam.ir_led(request.on)
  mutex.release()
  return irResponse(success)

def gpio_callback(request): # GPIO config service
  global omv_cam
  global mutex
  global pub_inputs
  mutex.acquire()
  success = omv_cam.gpio_config(request.nr, request.output, request.opendrain, request.pullup, request.pulldown)
  mutex.release()
  if success:
    found = False
    for pub in pub_inputs:
      if pub[0] == request.nr:
        found = True
        if request.output:
          pub[1].unregister()
          pub_inputs.remove(pub)
        break
    if not found and not request.output:
      pub_inputs.append([request.nr, rospy.Publisher(rospy.get_name() + "/input_" + str(request.nr), Bool, queue_size = 1)]) 
  return gpioResponse(success)

def gpio_set_callback(request): # GPIO output service
  global omv_cam
  global mutex
  mutex.acquire()
  success = omv_cam.gpio_set(request.nr, request.on)
  mutex.release()
  return gpio_setResponse(success)

def main():
  global omv_cam
  global pub_inputs
  # Initialize node, camera, services, publishers and subscribers with parameters.
  rospy.init_node('openmv')

  frame_id = rospy.get_param("~frame_id", "openmv_camera_frame")

  show = rospy.get_param("~show_image", False)

  # Configure rates with precision of 100 Hz
  rate = rospy.Rate(100)
  frame_rate = int(100 / rospy.get_param("~frames_hz", 10))
  frame_rate_cnt = 0
  gpio_rate = int(100 / rospy.get_param("~gpio_hz", 1))
  gpio_rate_cnt = 0

  omv_cam = OpenMvInterface(rospy.get_param("~port", "/dev/ttyACM0"),
     rospy.get_param("~resolution", "QQVGA"))

  service_rgb = rospy.Service(rospy.get_name() + '/rgb', rgb, rgb_callback)
  service_ir = rospy.Service(rospy.get_name() + '/ir', ir, ir_callback)
  service_gpio = rospy.Service(rospy.get_name() + '/gpio_config', gpio, gpio_callback)
  service_gpio_set = rospy.Service(rospy.get_name() + '/gpio_set', gpio_set, gpio_set_callback)

  img_name = rospy.get_name() + "/" + rospy.get_param("~image_topic", "image_color")
 
  pub_image = rospy.Publisher(img_name,
          Image, queue_size = rospy.get_param("~image_queue", 1))

  pub_image_compressed = rospy.Publisher(img_name + "_compressed",
          CompressedImage, queue_size = rospy.get_param("~image_queue", 1))

  pub_info = rospy.Publisher(rospy.get_name() + "/" + rospy.get_param("~info_topic", "camera_info"),
          CameraInfo, queue_size = rospy.get_param("~image_queue", 1))

  # Initialize messages.
  img_msg = Image()
  img_msg.header.frame_id = frame_id
  img_compressed_msg = CompressedImage()
  img_compressed_msg.header.frame_id = frame_id

  info_msg = CameraInfo()
  info_msg.header.frame_id = frame_id

  global mutex
  while not rospy.is_shutdown():
    mutex.acquire()
   
    if frame_rate_cnt > frame_rate:
        
        frame_rate_cnt = 0

        framedump = omv_cam.dump() # Get new frame from camera buffer.
        if framedump is not None:
          if show: # Show if required.
            cv2.imshow(img_name, framedump.pixels)
            cv2.waitKey(1)
          # Publish what we have.
          pub_image.publish(framedump.fill_img_msg(img_msg))
          pub_image_compressed.publish(framedump.fill_jpeg_msg(img_compressed_msg))
          pub_info.publish(framedump.fill_info_msg(info_msg))

    if gpio_rate_cnt > gpio_rate:
        
        gpio_rate_cnt = 0
       
        omv_cam.gpio_poll_inputs()
    
        for gpio_input in omv_cam.inputs:
          for pub in pub_inputs:
            if pub[0] == gpio_input[0]:
              pub[1].publish(bool(gpio_input[1]))
              break
   
    mutex.release()
    
    frame_rate_cnt += 1
    gpio_rate_cnt += 1
    rate.sleep()

if __name__ == '__main__':
  main()
