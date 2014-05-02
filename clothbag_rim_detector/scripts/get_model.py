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

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, PointCloud2, CameraInfo

import threading


def camera_info_callback(msg_camera_info):
  global camera_info, camera_info_flag, time_start, depth_plane_model
  try:
    camera_info_flag
    return
  except:
    camera_info = msg_camera_info
    rospy.loginfo("Subscribe camera_info has finished.")
    # finish subscribe
    time_start = time.time()
    camera_info_flag = True
    depth_plane_model = np.zeros((camera_info.height, camera_info.width), dtype=np.float64)
    rospy.Timer(rospy.Duration(0.1), timer_callback)


def timer_callback(event):
  global depth_world, depth_plane_model, last_time, count, camera_info, rospy_thread
  try:
    camera_info
  except:
    rospy.loginfo("Waiting for camera info...")
    return
  cv2.waitKey(1)
  time_now = time.time()
  if time_now - time_start < 1:
    last_time = 0
    count = 0
    return
  elif last_time < time.time():
    rospy.loginfo("add to plane_model")
    if not depth_plane_model.any():
      depth_plane_model = depth_world
    else:
      diff = depth_plane_model - depth_world
      depth_plane_model = (depth_plane_model + depth_world) / 2
      depth_plane_model[abs(diff) > 100] = 0
    last_time = int(time.time()) + 1
    count += 1
  elif count > 5:
    rospy.loginfo("Getting plane_model has finished.")
    # save
    np.save(os.path.dirname(sys.argv[0])+"/depth_plane_model", depth_plane_model)
    cv2.imwrite(sys.argv[0]+"/depth_plane_model.jpg", depth_plane_model)
    exit()
  cv2.imshow("depth_plane_model", depth2gray(depth_plane_model))


def xyz_world_callback(msg_xyz):
  global depth_world
  try:
    cv_bridge = CvBridge()
    xyz_world = np.asarray(cv_bridge.imgmsg_to_cv(msg_xyz, "32FC3"))
    depth_world = xyz_world[:,:,2] * 1000.0
    cv2.imshow("depth_world", depth2gray(depth_world))
  except CvBridgeError as e:
    rospy.logerr(e)


def sleep():
  time.sleep(7)


def depth2gray(depth, min_mm=300, max_mm=1000, margin=10):
  # depth max is 4095
  try:
    gray = depth.copy()
    if min_mm < gray[gray.nonzero()].min():
      min_mm = gray[gray.nonzero()].min()
    if gray.max() < max_mm:
      max_mm = gray.max()
    gray[(gray<min_mm) | (max_mm<gray)] = 0
    gray[gray.nonzero()] = (gray[gray.nonzero()]-min_mm)/(max_mm-min_mm)*(255-margin) + margin
    return np.uint8(gray)
  except:
    return np.zeros(depth.shape, dtype=np.uint8)


if __name__ == '__main__':
  global rospy_thread
  rospy.init_node("detect_rim", anonymous=True)
  sub_camera_info = rospy.Subscriber("/camera/depth_registered/camera_info",
                                     CameraInfo,
                                     camera_info_callback)
  sub_xyz_world = rospy.Subscriber("/my_camera/mat/xyz_world",
                                   Image,
                                   xyz_world_callback)

  rospy_thread = threading.Thread(target = rospy.spin)
  rospy_thread.setDaemon(True)
  rospy_thread.start()

  sleep_thread = threading.Thread(target = sleep)
  sleep_thread.setDaemon(False)
  sleep_thread.start()
