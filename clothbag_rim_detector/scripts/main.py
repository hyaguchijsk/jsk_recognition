#!/usr/bin/env python

import sys
import os
import time
import commands

import rospy
# import roslib
import rospkg
import tf

from clothbag_rim_detector.camera import *

PLANE_HEIGHT = 725.5/1000.0 # meter

class NullClass():
  pass

def redef():
  pkgpath = rospkg.RosPack().get_path('clothbag_rim_detector')
  execfile(pkgpath + "/src/clothbag_rim_detector/process.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/draw.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/utils.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/develop.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/tmp.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/filter.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/test_contour.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/hist.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/rim.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/target_point.py", globals())
  execfile(pkgpath + "/src/clothbag_rim_detector/param.py", globals())

def dump_scene():
  global camera, scene
  scene = camera.dump()

# timer_callback
def process_wrap(event):
  global camera
  if not camera.camera_info_flag:
    rospy.loginfo("Waiting for camera info...")
    return
  try:
    camera.transform
  except:
    rospy.loginfo("Waiting for camera transform...")
    return

  process()

  cv2.waitKey(1)

if __name__ == '__main__':
  redef()
  rospy.init_node("detect_rim", anonymous=True)
  camera = Camera()
  global pub_color_edit, bridge
  pub_color_edit = rospy.Publisher("/detect_rim/color_edit", Image)
  bridge = CvBridge()

  process_hz = 5
  rospy.Timer(rospy.Duration(1.0/process_hz), process_wrap)

  rospy_thread = threading.Thread(target = rospy.spin)
  rospy_thread.setDaemon(True)
  rospy_thread.start()


