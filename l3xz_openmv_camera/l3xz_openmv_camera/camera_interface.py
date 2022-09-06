'''
This software is distributed under the terms of the MIT License.
Copyright (c) 2022 107-Systems
Author: Jonas Wühr <jonaswuehrmaintainer@gmail.com>
'''
import cv2
import numpy as np
import struct 
import io, serial, serial.tools.list_ports, socket, sys

import rclpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage 
from sensor_msgs.msg import CameraInfo 

import struct 
from openmv.tools.rpc import rpc 

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
    self._stamp = rclpy.clock.Clock().now()

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
    msg.header.stamp = self._stamp.to_msg()
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
    msg.header.stamp = self._stamp.to_msg()
    msg.format = "jpeg"
    msg.data = self._jpeg.tostring()
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
    msg.header.stamp = self._stamp.to_msg()
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
    assert resolution == "QVGA" or resolution == "QQVGA"
    self._resolution = resolution
    self._interface = rpc.rpc_usb_vcp_master(port) # Connect via rpc.
    self._inputs = []

  def __del__(self):
    pass

  @property
  def inputs(self):
    return self._inputs

  def rgb_led(self, r, g, b):
    """Sets RGB LED
  
    Args:
      r: Enable red.
      g: Enable green.
      b: Enable blue.

    Returns:
      True for success
    """
    result = self._interface.call("rgb", struct.pack("<bbb", r, g, b))
    return result is not None
  
  def ir_led(self, enable):
    """Sets IR LED
  
    Args:
      enable: Enable led.
    
    Returns:
      True for success
    """
    data = ""
    if enable:
      data = "1"
    else:
      data = "0"

    result = self._interface.call("ir", data)
    return result is not None

  def gpio_config(self, pin, output, opendrain, pullup, pulldown):
    """Configures GPIO
  
    Args:
      pin: Pin number from P0 to P9
      output: Input/ Output
      opendrain
      pullup
      pulldown
    
    Returns:
      True for success
    """
    assert (not pullup and not pulldown) or (pullup != pulldown) or opendrain 
    result = self._interface.call("gpio_config", struct.pack("<bbbbb", pin, output, opendrain, pullup, pulldown))
    if result is not None:
      for i in self._inputs:
        if pin == i[0]:
          self._inputs.remove(i)
      if not output:
        self._inputs.append([pin, False])
      return True
    else:
      return False

  def gpio_set(self, pin, value):
    """Set GPIO output
  
    Args:
      pin: Pin number from P0 to P9
      output: High/ low 
    
    Returns:
      True for success
    """
    result = self._interface.call("gpio_set", struct.pack("<bb", pin, value))
    return result is not None
 
  def gpio_poll(self, pin):
    """Poll GPIO input
  
    Args:
      pin: Pin number from P0 to P9
    
    Returns:
      High/ low 
    """
    result = self._interface.call("gpio_poll", str(pin)) 
    if result:
      val = struct.unpack("<b", result)[0]
    else:
      val = -1
    return val
  
  def gpio_poll_inputs(self):
    """Polls all configured GPIOs."""
    for gpio in self._inputs:
      gpio[1] = self.gpio_poll(gpio[0])

  def dump(self):
    """Dumps RGB565 frame buffer from device.
  
    Returns:
      FrameDump object with image.
    """
    result = self._interface.call("jpeg_image_snapshot", "sensor.RGB565,sensor." + self._resolution) # Initiate snapshot.
    
    if result is not None:
      size = struct.unpack("<I", result)[0]
      raw = bytearray(size)
      isData = self._interface.call("jpeg_image_read") # Fill buffer.
      
      if isData is not None:
        # Read and decode buffer with jpeg data.
        self._interface.get_bytes(raw, 5) 
        np_img = np.frombuffer(raw, dtype=np.uint8)
        img = cv2.imdecode(np_img, cv2.IMREAD_UNCHANGED)
        return FrameDump(img, np_img)

    return None
