/// @file point_cloud_to_cv_mat.hpp
/// @brief converting pcl::PointCloud to cv::Mat
/// @author Kenji Miyake, Hiroaki Yaguchi

#ifndef CLOTHBAG_RIM_DETECTOR_POINT_CLOUD_TO_CV_MAT_HPP_
#define CLOTHBAG_RIM_DETECTOR_POINT_CLOUD_TO_CV_MAT_HPP_

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

namespace clothbag_rim_detector {

// typedef pcl::PointXYZRGBNormal point_type;
typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> cloud_type;
typedef pcl::PointCloud<point_type>::Ptr cloud_type_ptr;

/// @brief PointCloudToCvMat: Publish Mat images of PointCloud
class PointCloudToCvMat {
 public:
  // Mat
  cv::Mat color;
  cv::Mat depth;
  cv::Mat mask;
  cv::Mat xyz_camera;
  cv::Mat xyz_world;
  cv::Mat normal_camera;
  cv::Mat normal_world;

  PointCloudToCvMat(std::string topic_name);
  void Publish(cloud_type cloud, cloud_type cloud_world);

 private:
  ros::NodeHandle nh;
  // cv_bridge
  cv_bridge::CvImage bridge;

  // Publisher
  // ros::Publisher pub_color;
  // ros::Publisher pub_depth;
  // ros::Publisher pub_mask;
  ros::Publisher pub_xyz_camera;
  ros::Publisher pub_xyz_world;
  ros::Publisher pub_normal_camera;
  ros::Publisher pub_normal_world;
  // Create Mat from PointCloud
  cv::Mat CreateMatXYZ_(cloud_type cloud);
  cv::Mat CreateMatNormal_(cloud_type cloud);

  cv::Mat CreateMatColor_(cloud_type cloud);
  cv::Mat CreateMatDepth_(cloud_type cloud);
  cv::Mat CreateMatMask_(cloud_type cloud);
};


}  // namespace


#endif  // CLOTHBAG_RIM_DETECTOR_POINT_CLOUD_TO_CV_MAT_HPP_
