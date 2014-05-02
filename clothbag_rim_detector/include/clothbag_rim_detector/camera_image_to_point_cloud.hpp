/// @file camera_image_to_point_cloud.hpp
/// @brief converter class from sensor_msgs::Image to pcl::PointCloud
/// @author Kenji Miyake, Hiroaki Yaguchi

#ifndef CLOTHBAG_RIM_DETECTOR_CAMERA_IMAGE_TO_POINT_CLOUD_HPP_
#define CLOTHBAG_RIM_DETECTOR_CAMERA_IMAGE_TO_POINT_CLOUD_HPP_

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>

#include "clothbag_rim_detector/point_cloud_to_cv_mat.hpp"

namespace clothbag_rim_detector {

/// @brief CameraImageToPointCloud:
class CameraImageToPointCloud {
 public:
  cv::Mat color, depth;
  cloud_type cloud;
  cloud_type cloud_world;

  sensor_msgs::CameraInfo camera_info;
  bool camera_info_flag;
  tf::StampedTransform stamped_transform;
  // Eigen::Transform<Scalar, 3, Eigen::Affine> transform_to_world;

  CameraImageToPointCloud();
  ~CameraImageToPointCloud();

  cloud_type createCloud(cv::Mat color, cv::Mat depth);
  void publishCloud(ros::Publisher pub_cloud, cloud_type cloud);

 private:
  ros::NodeHandle nh;
  void init();

  // Timer
  ros::Timer timer;

  // Subscriber
  ros::Subscriber sub_color;
  ros::Subscriber sub_depth;
  ros::Subscriber sub_cloud;
  ros::Subscriber sub_camera_info;

  tf::TransformListener tf_listener;

  // Publisher
  ros::Publisher pub_cloud;

  PointCloudToCvMat mat_set;

  // callback functions
  void colorCallback(const sensor_msgs::ImageConstPtr &msg_color);
  void depthCallback(const sensor_msgs::ImageConstPtr &msg_depth);
  // void cloudCallback(const sensor_msgs::PointCloud2 &msg_cloud);
  void cameraInfoCallback(
      const sensor_msgs::CameraInfoConstPtr &msg_camera_info);

  void timerCallback(const ros::TimerEvent &event);
  void computeDepthWorld();

};


}  // namespace

#endif  // CLOTHBAG_RIM_DETECTOR_CAMERA_IMAGE_TO_POINT_CLOUD_HPP_
