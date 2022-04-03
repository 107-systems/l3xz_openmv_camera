import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage 
from sensor_msgs.msg import CameraInfo 

import cv2
import numpy as np

import struct 
from openmv.tools import pyopenmv
from openmv.tools.rpc import rpc 

import io, serial, serial.tools.list_ports, socket, sys
from enum import Enum

class FrameDump:

  def __init__(self, image, jpeg = None):
    self._height, self._width = image.shape[:2]
    self._pixels = image
    self._jpeg = jpeg
    self._stamp = rospy.Time.now()

  @property 
  def width(self):
    return self._width

  @property
  def height(self):
    return self._height

  @property
  def pixels(self):
    return self._pixels

  def fill_img_msg(self, msg = None):
    if msg is None:
      msg = Image()
    msg.header.stamp = self._stamp 
    msg.encoding = "bgr8"
    msg.step = 3 * self._width 
    msg.width = self._width 
    msg.height = self._height
    msg.data = self._pixels.tostring()
    return msg

  def fill_jpeg_msg(self, msg = None):
    if msg is None:
      msg = CompressedImage()
    msg.header.stamp = self._stamp
    msg.format = "jpeg"
    msg.data = self._jpeg
    return msg

  def fill_info_msg(self, msg = None):
    if msg is None:
      msg = CameraInfo()
    msg.header.stamp = self._stamp
    msg.width = self._width
    msg.height = self._height
    return msg

class OpenMvInterface:

  def __init__(self, port, script_file, resolution = "QQVGA"):
    self._resolution = resolution
    self._interface = rpc.rpc_usb_vcp_master(port)

  def __del__(self):
    pass

  def rgb_led(self, r, g, b):
    data = ""
    if r:
      data += "1,"
    else:
      data += "0,"
    if g:
      data += "1,"
    else:
      data += "0,"
    if b:
      data += "1"
    else:
      data += "0"
 
    result = self._interface.call("rgb", data)
    return result is not None
 
  def dump(self):
    result = self._interface.call("jpeg_image_snapshot", "sensor.RGB565,sensor." + self._resolution)
    size = struct.unpack("<I", result)[0]
    raw = bytearray(size)
    result = self._interface.call("jpeg_image_read")
    
    if result is not None:
      self._interface.get_bytes(raw, 5000)
      np_img = np.frombuffer(raw, dtype=np.uint8)
      img = cv2.imdecode(np_img, cv2.IMREAD_UNCHANGED)
      return FrameDump(img, np_img)

    return None
