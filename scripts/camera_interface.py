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

class FrameDump:

  def __init__(self, image, jpeg = None):
    """Fills data of frame.
   
    Args:
      image: Image as OpenCV numpy array.
      jpeg: JPEG stream from camera.
    """
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
    """Fills data to ROS image message.
  
    Args:
      msg: sensor_msgs/Image
    
    Returns:
      Filled message.
    """
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
    """Fills data to ROS compressed image message.
  
    Args:
      msg: sensor_msgs/CompressedImage
    
    Returns:
      Filled message.
    """
    if msg is None:
      msg = CompressedImage()
    msg.header.stamp = self._stamp
    msg.format = "jpeg"
    msg.data = self._jpeg
    return msg

  def fill_info_msg(self, msg = None):
    """Fills data to ROS camera info message.
  
    Args:
      msg: sensor_msgs/CameraInfo
    
    Returns:
      Filled message.
    """
    if msg is None:
      msg = CameraInfo()
    msg.header.stamp = self._stamp
    msg.width = self._width
    msg.height = self._height
    return msg

class OpenMvInterface:

  def __init__(self, port, resolution = "QQVGA"):
    """Interfaces camera via openmv rpc calls.
   
    Args:
      port: Serial port device.
      resolution: Camera resolution (QQVGA or QVGA)
    """
    self._resolution = resolution
    self._interface = rpc.rpc_usb_vcp_master(port) # Connect via rpc.

  def __del__(self):
    pass

  def rgb_led(self, r, g, b):
    """Sets RGB LED
  
    Args:
      r: Enable red.
      g: Enable green.
      b: Enable blue.
    """
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
 
    result = self._interface.call("rgb", data) # Do rpc call.
    return result is not None
 
  def dump(self):
    """Dumps RGB565 frame buffer from device.
  
    Returns:
      FrameDump object with image.
    """
    result = self._interface.call("jpeg_image_snapshot", "sensor.RGB565,sensor." + self._resolution) # Initiate snapshot.
    size = struct.unpack("<I", result)[0]
    raw = bytearray(size)
    result = self._interface.call("jpeg_image_read") # Fill buffer.
    
    if result is not None:
      # Read and decode buffer with jpeg data.
      self._interface.get_bytes(raw, 5000) 
      np_img = np.frombuffer(raw, dtype=np.uint8)
      img = cv2.imdecode(np_img, cv2.IMREAD_UNCHANGED)
      return FrameDump(img, np_img)

    return None
