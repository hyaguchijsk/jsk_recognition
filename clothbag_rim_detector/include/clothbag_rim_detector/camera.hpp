/// @file camera.hpp
/// @brief
/// @author Kenji Miyake, Hiroaki Yaguchi

#ifndef CLOTHBAG_RIM_DETECTOR_CAMERA_HPP_
#define CLOTHBAG_RIM_DETECTOR_CAMERA_HPP_

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


/// @brief MatSet: Publish Mat images of PointCloud
class MatSet {
 public:
  // Mat
  cv::Mat color;
  cv::Mat depth;
  cv::Mat mask;
  cv::Mat xyz_camera;
  cv::Mat xyz_world;
  cv::Mat normal_camera;
  cv::Mat normal_world;

  MatSet(std::string topic_name);
  void publish(cloud_type cloud, cloud_type cloud_world);

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
  cv::Mat createMatXYZ(cloud_type cloud);
  cv::Mat createMatNormal(cloud_type cloud);

  cv::Mat createMatColor(cloud_type cloud);
  cv::Mat createMatDepth(cloud_type cloud);
  cv::Mat createMatMask(cloud_type cloud);
};


/// @brief Camera: import this from main program,and use camera.color camera.depth to get images
class Camera {
 public:
  cv::Mat color, depth;
  cloud_type cloud;
  cloud_type cloud_world;

  sensor_msgs::CameraInfo camera_info;
  bool camera_info_flag;
  tf::StampedTransform stamped_transform;
  // Eigen::Transform<Scalar, 3, Eigen::Affine> transform_to_world;

  Camera();
  ~Camera();

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

  MatSet mat_set;

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

#endif  // CLOTHBAG_RIM_DETECTOR_CAMERA_HPP_
