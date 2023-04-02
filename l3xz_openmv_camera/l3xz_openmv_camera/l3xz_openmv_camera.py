'''
This software is distributed under the terms of the MIT License.
Copyright (c) 2022 107-Systems
Author: Jonas Wühr <jonaswuehrmaintainer@gmail.com>
'''
import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import threading
import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage 
from sensor_msgs.msg import CameraInfo 

from l3xz_openmv_camera_interfaces.srv import Rgb, Ir, Gpio, GpioSet
from camera_interface import OpenMvInterface

class CameraNode(Node):

  def __init__(self):
    super().__init__('l3xz_openmv_camera')
    self.declare_parameters(
      namespace = '',
      parameters = [
        ('image_topic', 'image_color'),
        ('image_queue', 1),
        ('info_topic', 'camera_info'),
        ('info_queue', 1),
        ('show_image', True),
        ('port', '/dev/ttyACM0'),
        ('frames_hz', 10),
        ('gpio_hz', 1),
        ('frame_id', 'openmv_camera_frame'),
        ('resolution', 'QQVGA')
      ]
    )
    
    self._cam = OpenMvInterface(self.get_parameter('port').value, self.get_parameter('resolution').value)
    
    self._img_msg = Image()
    self._img_msg.header.frame_id = self.get_parameter('frame_id').value
    self._img_compressed_msg = CompressedImage()
    self._img_compressed_msg.header.frame_id = self.get_parameter('frame_id').value
    self._info_msg = CameraInfo()
    self._info_msg.header.frame_id = self.get_parameter('frame_id').value

    self._pub_image = self.create_publisher(Image, self.get_name() + "/" + self.get_parameter('image_topic').value, self.get_parameter('image_queue').value)
    self._pub_image_compressed = self.create_publisher(CompressedImage, self.get_name() + "/" +self.get_parameter('image_topic').value + "_compressed", self.get_parameter('image_queue').value)
    self._pub_info = self.create_publisher(CameraInfo, self.get_name() + "/" + self.get_parameter('info_topic').value, self.get_parameter('image_queue').value)
    img_timer_period = 1.0 / self.get_parameter('frames_hz').value
    self._img_timer = self.create_timer(img_timer_period, self._img_timer_callback)

    self._srv_rgb = self.create_service(Rgb, self.get_name() + '/rgb', self._rgb_callback)
    self._srv_ir = self.create_service(Ir, self.get_name() + '/ir', self._ir_callback)

    self._pub_inputs = []
    self._srv_gpio = self.create_service(Gpio, self.get_name() + '/gpio_config', self._gpio_callback)
    self._srv_gpio_set = self.create_service(GpioSet, self.get_name() + '/gpio_set', self._gpio_set_callback)
    gpio_timer_period = 1.0 / self.get_parameter('gpio_hz').value
    self._gpio_timer = self.create_timer(gpio_timer_period, self._gpio_timer_callback)

  def _img_timer_callback(self):
    framedump = self._cam.dump()
    if framedump is not None:
      if self.get_parameter('show_image').value:
        cv2.imshow(self.get_name(), framedump.pixels)
        cv2.waitKey(1)
      self._pub_image.publish(framedump.fill_img_msg(self._img_msg))
      self._pub_image_compressed.publish(framedump.fill_jpeg_msg(self._img_compressed_msg))
      self._pub_info.publish(framedump.fill_info_msg(self._info_msg))

  def _rgb_callback(self, request, response):
    response.success = self._cam.rgb_led(request.r, request.g, request.b)
    return response

  def _ir_callback(self, request, response):
    response.success = self._cam.ir_led(request.on)
    return response

  def _gpio_callback(self, request, response):
    response.success = self._cam.gpio_config(request.nr, request.output, request.opendrain, request.pullup, request.pulldown)
    if response.success:
      self._pub_inputs.append([request.nr, self.create_publisher(Bool, self.get_name() + "/input_" + str(request.nr), 1)]) 
    return response

  def _gpio_set_callback(self, request, response):
   response.success = self._cam.gpio_set(request.nr, request.on)
   return response

  def _gpio_timer_callback(self):
    self._cam.gpio_poll_inputs()
    for gpio_input in self._cam.inputs:
      for pub in self._pub_inputs:
        if pub[0] == gpio_input[0]:
          msg = Bool()
          msg.data = bool(gpio_input[1])
          pub[1].publish(msg)
          break

def main(args=None):
  rclpy.init(args=args)

  node = CameraNode()

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
