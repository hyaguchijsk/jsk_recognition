#!/usr/bin/env python
# -*- coding: utf-8 -*-

## @package clothbag_rim_detector
# Obtaining table model
#
# @author Hiroaki Yaguchi - h-yaguchi@jsk.t.u-tokyo.ac.jp

import os
import sys
import time

import rospy
import rospkg

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, PointCloud2, CameraInfo

import threading

class get_table_model():
  def __init__(self):
    self.time_start = time.time()
    self.camera_info = False
    self.camera_info_flag = False
    self.last_time = 0
    self.count = 0

  def camera_info_callback(self, msg_camera_info):
    try:
      self.camera_info_flag
      return
    except:
      self.camera_info = msg_camera_info
      rospy.loginfo("Subscribe camera_info has finished.")
      # finish subscribe
      self.time_start = time.time()
      camera_info_flag = True
      self.depth_plane_model = np.zeros(
        (camera_info.height, camera_info.width),
        dtype = np.float64)
      rospy.Timer(rospy.Duration(0.1), timer_callback)

  def timer_callback(self, event):
    try:
      self.camera_info
    except:
      rospy.loginfo("Waiting for camera info...")
      return
    cv2.waitKey(1)

    time_now = time.time()
    if time_now - self.time_start < 1:
      self.last_time = 0
      self.count = 0
      return
    elif self.last_time < time.time():
      rospy.loginfo("add to plane_model")
      if not self.depth_plane_model.any():
        self.depth_plane_model = self.depth_world
      else:
        diff = self.depth_plane_model - self.depth_world
        self.depth_plane_model = (
          self.depth_plane_model + self.depth_world) / 2
        self.depth_plane_model[abs(diff) > 100] = 0
      self.last_time = int(time.time()) + 1
      count += 1
    elif count > 5:
      rospy.loginfo("Getting plane_model has finished.")

      # save
      pkgpath = rospkg.get_path('clothbag_rim_detector')
      np.save(
        # os.path.dirname(sys.argv[0]) + "/depth_plane_model",
        pkgpath + "/model/depth_plane_model",
        self.depth_plane_model)
      cv2.imwrite(
        # sys.argv[0] + "/depth_plane_model.jpg",
        pkgpath + "/model/depth_plane_model.jpg",
        self.depth_plane_model)
      exit()
    cv2.imshow("depth_plane_model", depth2gray(self.depth_plane_model))


  def xyz_world_callback(self, msg_xyz):
    try:
      cv_bridge = CvBridge()
      xyz_world = np.asarray(cv_bridge.imgmsg_to_cv(msg_xyz, "32FC3"))
      self.depth_world = xyz_world[:,:,2] * 1000.0
      cv2.imshow("depth_world", depth2gray(self.depth_world))
    except CvBridgeError as e:
      rospy.logerr(e)


def sleep():
  for i in range(7):
    rospy.loginfo(str(i))
    time.sleep(1)
  rospy.loginfo("finished.")


def depth2gray(depth, min_mm = 300, max_mm = 1000, margin = 10):
  # depth max is 4095
  try:
    gray = depth.copy()
    if min_mm < gray[gray.nonzero()].min():
      min_mm = gray[gray.nonzero()].min()
    if gray.max() < max_mm:
      max_mm = gray.max()
    gray[(gray < min_mm) | (max_mm < gray)] = 0
    gray[gray.nonzero()] = (
      (gray[gray.nonzero()] - min_mm) / (max_mm - min_mm)
      * (255 - margin) + margin)
    return np.uint8(gray)
  except:
    return np.zeros(depth.shape, dtype = np.uint8)


if __name__ == '__main__':
  rospy.init_node("get_model", anonymous = True)
  get_model = get_table_model()

  sub_camera_info = rospy.Subscriber("/camera/depth_registered/camera_info",
                                     CameraInfo,
                                     get_model.camera_info_callback)
  sub_xyz_world = rospy.Subscriber("/my_camera/mat/xyz_world",
                                   Image,
                                   get_model.xyz_world_callback)

  rospy_thread = threading.Thread(target = rospy.spin)
  rospy_thread.setDaemon(True)
  rospy_thread.start()

  sleep_thread = threading.Thread(target = sleep)
  sleep_thread.setDaemon(False)
  sleep_thread.start()
