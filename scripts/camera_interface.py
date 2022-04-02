import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo 

import cv2
import numpy as np

from openmv.tools import pyopenmv

class FrameDump:

  def __init__(self, width, height, pixels):
    self._width = width
    self._height = height
    self._pixels = pixels
    
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
    msg.header.stamp = rospy.Time.now()
    msg.encoding = "bgr8"
    msg.step = 3 * self._width 
    msg.width = self._width 
    msg.height = self._height
    msg.data = self._pixels.tostring()
    return msg

  def fill_info_msg(self, msg = None):
    if msg is None:
      msg = CameraInfo()
    msg.width = self._width
    msg.height = self._height
    return msg

class OpenMvInterface:

  def __init__(self, port, script_file):
    pyopenmv.disconnect()
    pyopenmv.init(port, baudrate = 921600, timeout = 0.050)
    pyopenmv.set_timeout(1 * 2)
    pyopenmv.stop_script()
    pyopenmv.enable_fb(True)
    script = open(script_file, "r")
    script_str = script.read()
    script.close()
    pyopenmv.exec_script(script_str)

  def __del__(self):
    pyopenmv.stop_script()
    pyopenmv.disconnect()

  def dump(self):
    fb =  pyopenmv.fb_dump()
    if fb is not None:
      return FrameDump(fb[0], fb[1], fb[2])
    else:
      return None
