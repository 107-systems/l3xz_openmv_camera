import cv2
from openmv.tools import pyopenmv

CAM_SCRIPT = """
import sensor, image, time
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
while(True):
  img = sensor.snapshot()
  sensor.flush()
"""

class OpenmvDumper:

  def __init__(self, port = "/dev/ttyACM0"):
    pyopenmv.disconnect()
    pyopenmv.init(port, baudrate = 921600, timeout = 0.050)
    pyopenmv.set_timeout(1 * 2)
    pyopenmv.stop_script()
    pyopenmv.enable_fb(True)
    pyopenmv.exec_script(CAM_SCRIPT)
  def __del__(self):
    pyopenmv.stop_script()
    pyopenmv.disconnect()

  def dump(self):
    fb =  pyopenmv.fb_dump()
    if fb is not None:
      img = fb[2]
      return img
    else:
      return None
