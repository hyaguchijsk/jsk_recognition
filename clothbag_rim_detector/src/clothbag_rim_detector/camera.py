#!/usr/bin/env python
import sys
import os
import time
import commands

import rospy
import tf

import cv2
from cv_bridge import CvBridge, CvBridgeError

# import pcl

import numpy as np
import scipy
import scipy.optimize

from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image, PointCloud2, CameraInfo

# debug
import ipdb
import threading

class Camera():
  def __init__(self):
    self._cv_bridge = CvBridge()
    self.camera_info_flag = False
    self._sub_camera_info = rospy.Subscriber("/camera/depth_registered/camera_info", CameraInfo, self._camera_info_callback)

  def update_transform(self, event):
    ## np.dot(mat, camera_xyz) = world_xyz
    try:
      # camera to world
      self._tf_listener.waitForTransform(self._base_frame, self._camera_frame, rospy.Time(0), rospy.Duration(3))
      xyz, quat = self._tf_listener.lookupTransform(self._base_frame, self._camera_frame, rospy.Time(0))
      mat = tf.transformations.quaternion_matrix(quat)
      mat[:3,3] += xyz
      self.transform = mat
      # lhand to world
      lhand_frame = "LHAND_CENTER"
      self._tf_listener.waitForTransform(self._camera_frame, lhand_frame, rospy.Time(0), rospy.Duration(3))
      self.lhand_xyz = np.array(self._tf_listener.lookupTransform(self._camera_frame, lhand_frame, rospy.Time(0))[0])
      self.lhand_xyz_world = np.array(self._tf_listener.lookupTransform(self._base_frame, lhand_frame, rospy.Time(0))[0])
      self.lhand_point = self.xyz_world2point(np.array(self.lhand_xyz_world))
      # rhand to world
      rhand_frame = "RHAND_CENTER"
      self._tf_listener.waitForTransform(self._camera_frame, rhand_frame, rospy.Time(0), rospy.Duration(3))
      self.rhand_xyz = np.array(self._tf_listener.lookupTransform(self._camera_frame, rhand_frame, rospy.Time(0))[0])
      self.rhand_xyz_world = np.array(self._tf_listener.lookupTransform(self._base_frame, rhand_frame, rospy.Time(0))[0])
      self.rhand_point = self.xyz_world2point(self.rhand_xyz_world)
    except (tf.LookupException, tf.ConnectivityException, tf.Exception) as e:
      rospy.logerr("Camera.update_transform: error")
      rospy.logerr(e)


  def _init_after_camera_info(self):
    self._init_images()
    self._set_up_tf()

    # Subscriber
    self._sub_color= rospy.Subscriber("/camera/rgb/image_color", Image, self._color_callback)
    self._sub_depth = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self._depth_callback)
    self._sub_xyz_camera = rospy.Subscriber("/my_camera/mat/xyz_camera", Image, self._xyz_camera_callback)
    self._sub_xyz_world = rospy.Subscriber("/my_camera/mat/xyz_world", Image, self._xyz_world_callback)
    self._sub_normal_camera = rospy.Subscriber("/my_camera/mat/normal_camera", Image, self._normal_camera_callback)
    self._sub_normal_world = rospy.Subscriber("/my_camera/mat/normal_world", Image, self._normal_world_callback)

    # timer
    rospy.Timer(rospy.Duration(0.1), self.update_transform)


  def dump(self):
    pass
    # dump_to_"pickle_dump/camera_time.now().dump"

  def _camera_info_callback(self, msg_camera_info):
    self.camera_info = msg_camera_info
    rospy.loginfo("Subscribe camera_info has finished.")
    self.camera_info_flag = True
    self._init_after_camera_info()
    # finish subscribe
    self._sub_camera_info.unregister()


  def _init_images(self):
    # color and depth
    K = self.camera_info.K
    cx, cy, self.fx, self.fy = K[2], K[5], K[0], K[4]

    self.x_margin, self.y_margin = 0, 0 # turn off roi
    # self.x_margin, self.y_margin = 70, 40
    self.width = self.camera_info.width - 2*self.x_margin
    self.height = self.camera_info.height - 2*self.y_margin
    self.cx = cx - self.x_margin
    self.cy = cy - self.y_margin

    self.color = np.zeros((self.height, self.width,3), dtype=np.uint8)
    self.depth = np.zeros((self.height, self.width), dtype=np.float64)
    self.depth_world = np.zeros((self.height, self.width), dtype=np.float64)

    self.is_point_mask = np.zeros((self.height, self.width), dtype=np.bool)

    # PointCloud
    self.xyz_camera = np.zeros((self.height, self.width, 3), dtype=np.float64)
    self.xyz_world = np.zeros((self.height, self.width, 3), dtype=np.float64)


  def _set_up_tf(self):
    self._tf_listener = tf.TransformListener()
    self._tf_broadcaster = tf.TransformBroadcaster()
    self._camera_frame = "/camera_rgb_optical_frame"
    self._base_frame = "/BASE"
    rospy.loginfo("Succeccfully initialized tf")


  def get_camera2world_mat(self):
    ## np.dot(mat, camera_xyz) = world_xyz
    try:
      self._tf_listener.waitForTransform(self._base_frame, self._camera_frame, rospy.Time(0), rospy.Duration(3))
      xyz, quat = self._tf_listener.lookupTransform(self._base_frame, self._camera_frame, rospy.Time(0))
      mat = tf.transformations.quaternion_matrix(quat)
      mat[:3,3] += xyz
      return mat
    except (tf.LookupException, tf.ConnectivityException, tf.Exception) as e:
      rospy.logerr(e)
      rospy.logerr("Lookup tf has failed. return identity matrix")
      return np.identity(4)

  ## get where the robot hand is
  def get_hand_point(self, hand):
    hand_frame = hand[0].upper() + "HAND_CENTER"
    try:
      self._tf_listener.waitForTransform(self._camera_frame, hand_frame, rospy.Time(0), rospy.Duration(3))
      x, y, z = self._tf_listener.lookupTransform(self._camera_frame, hand_frame, rospy.Time(0))[0]
      px, py = self.get_2dpoint_from_3dpoint(x, y, z)
      return (px, py)
    except (tf.LookupException, tf.ConnectivityException, tf.Exception) as e:
      rospy.logerr(e)
      rospy.logerr("Lookup hand tf has failed. return (0, 0)")
      return (0, 0)

  def check_shape(self, point):
    img_shape = self.xyz_world.shape[:2]
    if ((point[0] < 0) or (img_shape[1] < point[0])) or ((point[1] < 0) or (img_shape[0] < point[1])):
      return False
    else:
      return True

  def point2xyz_world(self, point):
    # img_shape = self.xyz_world.shape[:2]
    # if ((point[0] < 0) or (img_shape[1] < point[0])) or ((point[1] < 0) or (img_shape[0] < point[1])):
    #   rospy.logerr("point2xyz_world: error, out of range")
    #   return np.array([0,0,0])
    return self.xyz_world[point[1], point[0]]

  def xyz_world2point(self, x, y=None, z=None):
    if x.__class__== np.ndarray and len(x)==3:
      tmp = x.copy()
      x = tmp[0]
      y = tmp[1]
      z = tmp[2]
    try:
      xyz_camera = np.dot(np.linalg.inv(self.transform), np.array((x,y,z,1)))
      return self.xyz2point(xyz_camera[0], xyz_camera[1], xyz_camera[2])
    except Exception as e:
      rospy.logerr(e)
      rospy.logerr("xyz_world2point: error")
      return (0,0)

  def xyz2point(self, x, y=None, z=None):
    if x.__class__== np.ndarray and len(x)==3:
      tmp = x.copy()
      x = tmp[0]
      y = tmp[1]
      z = tmp[2]
    d = 1000.0 * z
    px = int(self.cx + 1000.0*x/d*self.fx)
    py = int(self.cy + 1000.0*y/d*self.fy)
    return np.array((px, py))

  def get_roi(self, img):
    return img[self.y_margin:self.height+self.y_margin, self.x_margin:self.width+self.x_margin]

  def restore_roi(self, roi_img):
    if len(roi_img.shape) == 2:
      res = np.zeros((self.camera_info.height, self.camera_info.width), dtype=roi_img.dtype)
    else:
      res = np.zeros((self.camera_info.height, self.camera_info.width, roi_img.shape[2]), dtype=roi_img.dtype)

    res[self.y_margin:self.height+self.y_margin, self.x_margin:self.width+self.x_margin] = roi_img
    return res

  # callback functions of mat
  def _color_callback(self, msg_color):
    try:
      self.color = self.get_roi(np.asarray(self._cv_bridge.imgmsg_to_cv(msg_color, "bgr8")))
      self.color_hsv = cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)
    except CvBridgeError as e:
      rospy.logerr(e)

  def _depth_callback(self, msg_depth):
    try:
      self.depth = self.get_roi(np.asarray(self._cv_bridge.imgmsg_to_cv(msg_depth, "32FC1")))
      self.is_point_mask = (self.depth != 0)
    except CvBridgeError as e:
      rospy.logerr(e)

  def _xyz_camera_callback(self, msg_xyz):
    try:
      self.xyz_camera = self.get_roi(np.asarray(self._cv_bridge.imgmsg_to_cv(msg_xyz, "32FC3")))
    except CvBridgeError as e:
      rospy.logerr(e)

  def _xyz_world_callback(self, msg_xyz):
    try:
      self.xyz_world = self.get_roi(np.asarray(self._cv_bridge.imgmsg_to_cv(msg_xyz, "32FC3")))
      self.depth_world = self.xyz_world[:,:,2] * 1000.0
    except CvBridgeError as e:
      rospy.logerr(e)

  def _normal_camera_callback(self, msg_normal):
    try:
      self.normal_camera = self.get_roi(np.asarray(self._cv_bridge.imgmsg_to_cv(msg_normal, "32FC3")))
    except CvBridgeError as e:
      rospy.logerr(e)

  def _normal_world_callback(self, msg_normal):
    try:
      self.normal_world = self.get_roi(np.asarray(self._cv_bridge.imgmsg_to_cv(msg_normal, "32FC3")))
    except CvBridgeError as e:
      rospy.logerr(e)
