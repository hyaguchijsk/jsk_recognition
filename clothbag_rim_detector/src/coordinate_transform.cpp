#include "camera.cpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/visualization/cloud_viewer.h>

// typedef pcl::PointXYZRGBNormal point_type;
// typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_type;

class CoordinateTransform
{
  public:
    CoordinateTransform(): nh("coordinate_transform"), nh_in_plane(nh,"in_plane"), nh_on_plane(nh,"on_plane")
    {
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
    ~CoordinateTransform()
    {
      std::cerr << "CoordinateTransform instance has been deleted" << std::endl;
    };

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


void CoordinateTransform::timerCallback(const ros::TimerEvent &event)
{
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

cloud_type CoordinateTransform::test_segmentation(cloud_type cloud)
{
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

void CoordinateTransform::estimatePlaneModel(cloud_type cloud)
{
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coordinate_transform");

  CoordinateTransform ct;
  ros::spin();

  return 0;
}


