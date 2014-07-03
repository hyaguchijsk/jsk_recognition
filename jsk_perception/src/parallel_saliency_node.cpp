// @brief sample node to use oriented gradient
// @author Hiroaki Yaguchi, JSK

#include <vector>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "jsk_perception/parallel_saliency.hpp"

namespace jsk_perception {

class ParallelSaliencyNode {
 public:
  explicit ParallelSaliencyNode(const ros::NodeHandle& nh) :
      handle_(nh), image_transport_(nh) {
    std::string param_name = "block_size";
    block_size_ = 16;
    handle_.getParam(param_name, block_size_);
    image_sub_ = image_transport_.subscribe(
        "/camera/rgb/image_raw", 1,
        &ParallelSaliencyNode::imageCallback, this);
    cv::namedWindow("OrientedGradient");
    cv::namedWindow("ParallelSaliency");
  }

  ~ParallelSaliencyNode() {
  }

 private:
  ros::NodeHandle handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  int block_size_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat cv_img = cv_ptr->image;
    cv::Mat cv_gimg;
    cv::Mat cv_ogimg;
    cv::Mat cv_hsvimg;
    cv::Mat cv_ogout_img;
    cv::Mat cv_out_img;

    cv::cvtColor(cv_img, cv_gimg, CV_BGR2GRAY);
    calcOrientedGradient2D(cv_gimg, cv_ogimg);
    cvtOGtoHSV(cv_ogimg, cv_hsvimg);
    cv::cvtColor(cv_hsvimg, cv_ogout_img, CV_HSV2BGR);
    calcParallelSaliencyMap(cv_ogimg, cv_out_img, block_size_);
    ROS_INFO("done");
    cv::imshow("OrientedGradient", cv_ogout_img);
    cv::imshow("ParallelSaliency", cv_out_img);
    cv::waitKey(1);
  }
};

}  // namespace


int main(int argc, char** argv) {
  ros::init(argc, argv, "parallel_saliency");
  ros::NodeHandle handle("~");
  jsk_perception::ParallelSaliencyNode paranode(handle);
  ros::spin();
  return 0;
}
