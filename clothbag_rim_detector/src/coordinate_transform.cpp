/// @file coordinate_transform.cpp
/// @brief
/// @author Kenji Miyake, Hiroaki Yaguchi

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/visualization/cloud_viewer.h>

#include "clothbag_rim_detector/camera.hpp"
#include "clothbag_rim_detector/coordinate_transform.hpp"

namespace clothbag_rim_detector {


MatSet::MatSet(std::string topic_name) : nh() {
  // pub_color = nh.advertise<sensor_msgs::Image>(topic_name+"/mat/color", 10);
  // pub_depth = nh.advertise<sensor_msgs::Image>(topic_name+"/mat/depth", 10);
  // pub_mask = nh.advertise<sensor_msgs::Image>(topic_name+"/mat/mask", 10);
  pub_xyz_camera = nh.advertise<sensor_msgs::Image>(topic_name+"/mat/xyz_camera", 10);
  pub_normal_camera = nh.advertise<sensor_msgs::Image>(topic_name+"/mat/normal_camera", 10);
  pub_xyz_world = nh.advertise<sensor_msgs::Image>(topic_name+"/mat/xyz_world", 10);
  pub_normal_world = nh.advertise<sensor_msgs::Image>(topic_name+"/mat/normal_world", 10);
}


void MatSet::publish(cloud_type cloud, cloud_type cloud_world) {
  // color = createMatColor(cloud);
  // depth = createMatDepth(cloud);
  // mask = createMatMask(cloud);
  xyz_camera = createMatXYZ(cloud);
  normal_camera = createMatNormal(cloud);
  xyz_world  = createMatXYZ(cloud_world);
  normal_world  = createMatNormal(cloud_world);

  // bridge.image = color;
  // bridge.encoding = "bgr8";
  // pub_color.publish(bridge.toImageMsg());

  // bridge.image = depth;
  // bridge.encoding = "32FC1";
  // pub_depth.publish(bridge.toImageMsg());

  // bridge.image = mask;
  // bridge.encoding = "8UC1";
  // pub_mask.publish(bridge.toImageMsg());

  bridge.image = xyz_camera;
  bridge.encoding = "32FC3";
  pub_xyz_camera.publish(bridge.toImageMsg());

  bridge.image = xyz_world;
  bridge.encoding = "32FC3";
  pub_xyz_world.publish(bridge.toImageMsg());

  bridge.image = normal_camera;
  bridge.encoding = "32FC3";
  pub_normal_camera.publish(bridge.toImageMsg());

  bridge.image = normal_world;
  bridge.encoding = "32FC3";
  pub_normal_world.publish(bridge.toImageMsg());
}

cv::Mat MatSet::createMatXYZ(cloud_type cloud) {
  cv::Mat xyz = cv::Mat::zeros(cloud.height, cloud.width, CV_32FC3);
  for(int i=0;i<cloud.height;i++){
    for(int j=0;j<cloud.width;j++){
      xyz.at<cv::Vec3f>(i,j)[0] = cloud.at(j,i).x;
      xyz.at<cv::Vec3f>(i,j)[1] = cloud.at(j,i).y;
      xyz.at<cv::Vec3f>(i,j)[2] = cloud.at(j,i).z;
    }
  }
  return xyz;
}


cv::Mat MatSet::createMatNormal(cloud_type cloud) {
  cv::Mat normal = cv::Mat::zeros(cloud.height, cloud.width, CV_32FC3);
  for(int i=0;i<cloud.height;i++){
    for(int j=0;j<cloud.width;j++){
      // normal.at<cv::Vec3f>(i,j)[0] = cloud.at(j,i).normal_x;
      // normal.at<cv::Vec3f>(i,j)[1] = cloud.at(j,i).normal_y;
      // normal.at<cv::Vec3f>(i,j)[2] = cloud.at(j,i).normal_z;
    }
  }
  return normal;
}

cv::Mat MatSet::createMatMask(cloud_type cloud) {
  cv::Mat mask = cv::Mat::zeros(cloud.height, cloud.width, CV_8UC1);
  for(int i=0;i<cloud.height;i++){
    for(int j=0;j<cloud.width;j++){
      mask.at<unsigned char>(i,j) = (cloud.at(j,i).z == 0);
    }
  }
  return mask;
}

cv::Mat MatSet::createMatColor(cloud_type cloud) {
  cv::Mat color = cv::Mat::zeros(cloud.height, cloud.width,CV_8UC3);
  for(int i=0;i<cloud.height;i++){
    for(int j=0;j<cloud.width;j++){
      color.at<cv::Vec3b>(i,j)[2] = cloud.at(j,i).r;
      color.at<cv::Vec3b>(i,j)[1] = cloud.at(j,i).g;
      color.at<cv::Vec3b>(i,j)[0] = cloud.at(j,i).b;
    }
  }
  return color;
}

cv::Mat MatSet::createMatDepth(cloud_type cloud) {
  cv::Mat depth = cv::Mat::zeros(cloud.height, cloud.width,CV_32FC1);
  for(int i=0;i<cloud.height;i++){
    for(int j=0;j<cloud.width;j++){
      depth.at<float>(i,j) = cloud.at(j,i).z;
    }
  }
  return depth;
}


Camera::Camera() : nh(), mat_set("/my_camera") {
  std::cout << "Camera instance has been created" << std::endl;
  camera_info_flag = false;

  sub_camera_info = nh.subscribe("/camera/depth_registered/camera_info", 1, &Camera::cameraInfoCallback, this);
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/my_camera/points", 10);
};


Camera::~Camera() {
  std::cerr << "Camera instance has been deleted" << std::endl;
};


void Camera::init() {
  color = cv::Mat::zeros(camera_info.height, camera_info.width,CV_8UC3);
  depth = cv::Mat::zeros(camera_info.height, camera_info.width,CV_32FC1);
  // register callback functions
  timer = nh.createTimer(ros::Duration(0.1), &Camera::timerCallback, this);

  sub_color = nh.subscribe("camera/rgb/image_color", 1, &Camera::colorCallback, this);
  sub_depth = nh.subscribe("camera/depth_registered/image_raw", 1, &Camera::depthCallback, this);
  // sub_cloud = nh.subscribe("camera/depth_registered/points", 1, &Camera::cloudCallback, this);
}

void Camera::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg_camera_info) {
  if(camera_info_flag) return;
  camera_info = *msg_camera_info;

  // substitute to Matrix (don't using)
  int MAT_SIZE = 3*3;
  float array[MAT_SIZE];
  for(int i=0; i<MAT_SIZE; i++) array[i] = camera_info.K[i];
  Eigen::Matrix3f mat_K = Eigen::Map<Eigen::Matrix3f>(array);

  init();
  camera_info_flag = true;
  ROS_INFO("Subscribe camera_info has finished.");

}

void Camera::colorCallback(const sensor_msgs::ImageConstPtr &msg_color) {
  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(msg_color, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform color image.");
    return;
  }
  color = bridge->image;
}

void Camera::depthCallback(const sensor_msgs::ImageConstPtr &msg_depth) {
  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(msg_depth, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform depth image.");
    return;
  }
  depth = bridge->image;

  bool depth_filter_flag;
  ros::param::param<bool>("depth_filter_flag", depth_filter_flag, false);

  if(depth_filter_flag){
    // cv::Mat element = cv::Mat::ones(1,7,CV_8UC1);
    // cv2.getStructuringElement(cv2.MORPH_RECT,(3,3));
    // element = cv::Mat::eye(5,5, CV_8UC1);
    // cv::dilate(depth, depth, element, cv::Point(-1,-1), depth_filter_iteration);
    // cv::erode(depth, depth, element, cv::Point(-1,-1), depth_filter_iteration);

    int depth_filter_iteration;
    ros::param::param<int>("depth_filter_iteration", depth_filter_iteration, 10);
    cv::Mat depth_filtered = depth.clone();
    cv::dilate(depth_filtered, depth_filtered, cv::Mat(), cv::Point(-1,-1), depth_filter_iteration);
    cv::erode(depth_filtered, depth_filtered, cv::Mat(), cv::Point(-1,-1), depth_filter_iteration);

    int depth_filter_margin;
    ros::param::param<int>("depth_filter_margin", depth_filter_margin, 50);
    for(int i=depth_filter_margin;i<depth.rows-depth_filter_margin;i++){
      for(int j=depth_filter_margin;j<depth.cols-depth_filter_margin;j++){
        if(depth.at<float>(i,j)==0)
        {
          depth.at<float>(i,j) = depth_filtered.at<float>(i,j);
        }

      }
    }

  }
}


// void Camera::cloudCallback(const sensor_msgs::PointCloud2 &msg_cloud)
// {
  // pcl::PCLPointCloud2 cloud_tmp;
  // pcl_conversions::toPCL(msg_cloud, cloud_tmp);
  // pcl::fromPCLPointCloud2(cloud_tmp, cloud);
// }


void Camera::timerCallback(const ros::TimerEvent &event) {
  cloud = createCloud(color, depth);
  computeDepthWorld();

  publishCloud(pub_cloud, cloud);
  mat_set.publish(cloud, cloud_world);
}


void Camera::computeDepthWorld() {
  // transform
  try{
    // destination_frame, original_frame
    tf_listener.waitForTransform("/BASE", camera_info.header.frame_id, ros::Time(0), ros::Duration(10.0) );
    tf_listener.lookupTransform("/BASE", camera_info.header.frame_id, ros::Time(0), stamped_transform);
    // tf_listener.waitForTransform("camera_info.header.frame_id", "/BASE", ros::Time(0), ros::Duration(10.0) );
    // tf_listener.lookupTransform("camera_info.header.frame_id", "/BASE", ros::Time(0), stamped_transform);
  }

  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  // matrix = stamped_transform.getBasis()
  // pcl_ros::transformPointCloud
  Eigen::Matrix4f transform;
  transform(0, 0) = stamped_transform.getBasis()[0][0];
  transform(0, 1) = stamped_transform.getBasis()[0][1];
  transform(0, 2) = stamped_transform.getBasis()[0][2];
  transform(0, 3) = stamped_transform.getOrigin().x();
  transform(1, 0) = stamped_transform.getBasis()[1][0];
  transform(1, 1) = stamped_transform.getBasis()[1][1];
  transform(1, 2) = stamped_transform.getBasis()[1][2];
  transform(1, 3) = stamped_transform.getOrigin().y();
  transform(2, 0) = stamped_transform.getBasis()[2][0];
  transform(2, 1) = stamped_transform.getBasis()[2][1];
  transform(2, 2) = stamped_transform.getBasis()[2][2];
  transform(2, 3) = stamped_transform.getOrigin().z();
  transform(3, 0) = 0;
  transform(3, 1) = 0;
  transform(3, 2) = 0;
  transform(3, 3) = 1;

  cloud_type cloud_out(camera_info.width, camera_info.height);

  for(int i=0;i<cloud.height;i++){
    for(int j=0;j<cloud.width;j++){
      if(cloud.at(j,i).z == 0) continue;
      Eigen::Vector4f p(cloud.at(j,i).x, cloud.at(j,i).y, cloud.at(j,i).z, 1);
      Eigen::Vector4f p2 = transform * p;

      point_type point;
      point.x = p2[0];
      point.y = p2[1];
      point.z = p2[2];
      // point.r = cloud.at(j,i).r;
      // point.g = cloud.at(j,i).g;
      // point.b = cloud.at(j,i).b;
      // point.normal_x = cloud.at(j,i).normal_x;
      // point.normal_y = cloud.at(j,i).normal_y;
      // point.normal_z = cloud.at(j,i).normal_z;

      cloud_out.at(j,i) = point;
    }
  }
  cloud_world = cloud_out;
}


cloud_type Camera::createCloud(cv::Mat color, cv::Mat depth) {
  cloud_type cloud(camera_info.width, camera_info.height);
  if(camera_info_flag)
  {
    // depth_registered/image_raw stores mm depth data
    double min_mm = 200, max_mm = 1000;
    for(int i=0;i<depth.rows;i++){
      for(int j=0;j<depth.cols;j++){
        if(depth.at<float>(i,j)<min_mm || max_mm<depth.at<float>(i,j))continue;
        point_type point;
        // input values
        float d = depth.at<float>(i,j)/1000.0;
        point.x = (j-camera_info.K[2])*d/camera_info.K[0];
        point.y = (i-camera_info.K[5])*d/camera_info.K[4];
        point.z = d;
        point.r = color.at<cv::Vec3b>(i,j)[2];
        point.g = color.at<cv::Vec3b>(i,j)[1];
        point.b = color.at<cv::Vec3b>(i,j)[0];

        // point.normal_x = point.r;
        // point.normal_y = point.g;
        // point.normal_z = point.b;

        cloud.at(j,i) = point;
      }
    }
  }

  return cloud;
}


void Camera::publishCloud(ros::Publisher pub_cloud, cloud_type cloud) {
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::PCLPointCloud2 cloud_tmp;
  pcl::toPCLPointCloud2(cloud, cloud_tmp);

  pcl_conversions::fromPCL(cloud_tmp, msg_cloud);
  msg_cloud.header.frame_id = camera_info.header.frame_id;

  pub_cloud.publish(msg_cloud);
}


CoordinateTransform::CoordinateTransform(): nh("coordinate_transform"), nh_in_plane(nh,"in_plane"), nh_on_plane(nh,"on_plane") {
  std::cout << "CoordinateTransform instance has been created" << std::endl;
  timer = nh.createTimer(ros::Duration(0.1), &CoordinateTransform::timerCallback, this);

  // pub_cloud_raw = nh.advertise<sensor_msgs::PointCloud2>("all/points", 10);

  pub_color_in_plane = nh.advertise<sensor_msgs::PointCloud2>("in_plane/points", 10);
  pub_color_on_plane = nh.advertise<sensor_msgs::PointCloud2>("on_plane/points", 10);

  pub_depth_in_plane = nh.advertise<sensor_msgs::PointCloud2>("in_plane/points", 10);
  pub_depth_on_plane = nh.advertise<sensor_msgs::PointCloud2>("on_plane/points", 10);

  pub_cloud_in_plane = nh.advertise<sensor_msgs::PointCloud2>("in_plane/points", 10);
  pub_cloud_on_plane = nh.advertise<sensor_msgs::PointCloud2>("on_plane/points", 10);
}


CoordinateTransform::~CoordinateTransform() {
  std::cerr << "CoordinateTransform instance has been deleted" << std::endl;
};


void CoordinateTransform::timerCallback(const ros::TimerEvent &event) {
  if(!camera.camera_info_flag || camera.cloud.height==0)
  {
    ROS_INFO("Waiting for camera_info...");
    return;
  }
  // cv::imshow("camera.color", camera.color);
  // cv::waitKey(1);

  cloud_type cloud_raw(camera.cloud);

  // estimatePlaneModel(cloud_raw);
  // cloud_on_plane = test_segmentation(cloud_raw);
  // camera.publishCloud(pub_cloud_on_plane, cloud_on_plane);
  // camera.publishCloud(pub_cloud_in_plane, cloud_in_plane);

}

// filter if on_plane

cloud_type CoordinateTransform::test_segmentation(cloud_type cloud) {
  cloud_type::Ptr cloud_ptr (cloud.makeShared());

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<point_type> pass;
  pass.setInputCloud (cloud_ptr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::MinCutSegmentation<point_type> seg;
  seg.setInputCloud (cloud_ptr);
  seg.setIndices (indices);

  cloud_type::Ptr foreground_points(new cloud_type ());
  foreground_points->points.push_back(cloud_ptr->at(400,300));
  seg.setForegroundPoints (foreground_points);

  seg.setSigma (0.25);
  seg.setRadius (3.0433856);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (0.8);

  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters); // -> freeze
  std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
// pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr colored_cloud = seg.getColoredCloud ();

  return *colored_cloud;
}

void CoordinateTransform::estimatePlaneModel(cloud_type cloud) {
  // get inliers indices
  pcl::PointIndices inliers_indices;

  // set parameters to segment
  sac_seg.setOptimizeCoefficients(true);
  sac_seg.setModelType(pcl::SACMODEL_PLANE);
  sac_seg.setMethodType(pcl::SAC_RANSAC);
  sac_seg.setDistanceThreshold(0.005);
  sac_seg.setInputCloud(cloud.makeShared());
  sac_seg.segment(inliers_indices, plane_co);

  // get extracted cloud
  pcl::ExtractIndices<point_type> extract;
  pcl::PointCloud<point_type> inliers_points;
  pcl::PointCloud<point_type> outliers_points;

  extract.setInputCloud(cloud.makeShared());
  extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers_indices));
  extract.setNegative(false);

  extract.setNegative(false);
  extract.filter(cloud_in_plane);

  cloud_type cloud_negative;
  extract.setNegative(true);
  extract.filter(cloud_negative);

  cloud_type pc;
  for (int i=0; i<cloud_negative.points.size(); i++)
  {
    point_type pp;
    pp.x = cloud_negative.points[i].x;
    pp.y = cloud_negative.points[i].y;
    pp.z = cloud_negative.points[i].z;
    if(pp.x*plane_co.values[0] + pp.y*plane_co.values[1] + pp.z*plane_co.values[2] + plane_co.values[3] < 0)
    {
      pp.rgb = cloud_negative.points[i].rgb;
      pc.push_back(pp);
    }
  }

  cloud_on_plane = pc;
}

// cv::Mat get_xyz_camera_mat(){
//   return cv::Mat();
// }
//
// void getXYZfromPixel(float u,float v,float d,float *x,float *y,float *z)
// {
//   *x = 0.001*(u-320)*d/525;
//   *y = 0.001*(v-240)*d/525;
//   *z = 0.001*d;
// };
//
// void getPixelfromXYZ(int *u,int *v,float x,float y,float z)
// {
//   float d = 1000*z;
//   *u = int(1000*x*525/d + 320.0);
//   *v = int(1000*y*525/d + 240.0);
// };
//


}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coordinate_transform");

  clothbag_rim_detector::CoordinateTransform ct;
  ros::spin();

  return 0;
}


