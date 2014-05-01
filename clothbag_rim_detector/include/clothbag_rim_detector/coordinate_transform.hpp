/// @file coordinate_transform.hpp
/// @brief
/// @author Kenji Miyake, Hiroaki Yaguchi

#ifndef CLOTHBAG_RIM_DETECTOR_COORDINATE_TRANSFORM_HPP_
#define CLOTHBAG_RIM_DETECTOR_COORDINATE_TRANSFORM_HPP_

namespace clothbag_rim_detector {

class CoordinateTransform {
 public:
  CoordinateTransform();
  ~CoordinateTransform();

  Camera camera;
  cv::Mat xyz_camera, xyz_world;

  void estimatePlaneModel(cloud_type cloud);
  cloud_type test_segmentation(cloud_type cloud);

  cloud_type cloud_in_plane;
  cloud_type cloud_on_plane;

 private:
  // NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_in_plane;
  ros::NodeHandle nh_on_plane;

  // Color Publisher
  ros::Publisher  pub_color_in_plane;
  ros::Publisher  pub_color_on_plane;

  // Depth Publisher
  ros::Publisher  pub_depth_in_plane;
  ros::Publisher  pub_depth_on_plane;

  // Cloud Publisher
  ros::Publisher  pub_cloud_in_plane;
  ros::Publisher  pub_cloud_on_plane;

  pcl::ModelCoefficients plane_co;
  pcl::SACSegmentation<point_type> sac_seg;

  ros::Timer timer;
  cv::Mat tabletop_map;
  double th;

  void timerCallback(const ros::TimerEvent &event);
};


}  // namespace

#endif  // CLOTHBAG_RIM_DETECTOR_COORDINATE_TRANSFORM_HPP_
