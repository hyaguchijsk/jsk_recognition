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

#include "jsk_perception/opponent_color.hpp"
#include "jsk_perception/oriented_gradient.hpp"

namespace jsk_perception {

class OrientedGradientNode {
 public:
  explicit OrientedGradientNode(const ros::NodeHandle& nh) :
      handle_(nh), image_transport_(nh) {
    std::string param_name = "color_channel";
    channel_ = "gray";
    handle_.getParam(param_name, channel_);
    ROS_INFO("channel = %s" , channel_.c_str());

    image_sub_ = image_transport_.subscribe(
        "/camera/rgb/image_raw", 1,
        &OrientedGradientNode::imageCallback, this);
    cv::namedWindow("InputImage");
    cv::namedWindow("OrientedGradient");
  }

  ~OrientedGradientNode() {
  }

 private:
  ros::NodeHandle handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;

  std::string channel_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat cv_img = cv_ptr->image;

    // convert to gray scale
    cv::Mat cv_gimg;
    cv::cvtColor(cv_img, cv_gimg, CV_BGR2GRAY);

    // convert to opponent color
    cv::Mat cv_opp_img;
    std::vector<cv::Mat> cv_opp_channels;
    cvtOpponentColor(cv_img, cv_opp_img);
    cv::split(cv_opp_img, cv_opp_channels);

    // split to bgr
    std::vector<cv::Mat> cv_bgr_channels;
    cv::split(cv_img, cv_bgr_channels);

    // calcOrientedGradient() will write oriented gradient to
    // 2nd arg image as HSV format,
    // H is orientation and V is intensity
    cv::Mat cv_og_img;
    if (channel_ == "o1") {
      cv::imshow("InputImage", cv_opp_channels[0]);
      calcOrientedGradient(cv_opp_channels[0], cv_og_img);
    } else if (channel_ == "o2") {
      cv::imshow("InputImage", cv_opp_channels[1]);
      calcOrientedGradient(cv_opp_channels[1], cv_og_img);
    } else if (channel_ == "o3") {
      cv::imshow("InputImage", cv_opp_channels[2]);
      calcOrientedGradient(cv_opp_channels[2], cv_og_img);
    } else if (channel_ == "red") {
      cv::imshow("InputImage", cv_bgr_channels[2]);
      calcOrientedGradient(cv_bgr_channels[2], cv_og_img);
    } else if (channel_ == "green") {
      cv::imshow("InputImage", cv_bgr_channels[1]);
      calcOrientedGradient(cv_bgr_channels[1], cv_og_img);
    } else if (channel_ == "blue") {
      cv::imshow("InputImage", cv_bgr_channels[0]);
      calcOrientedGradient(cv_bgr_channels[0], cv_og_img);
    } else {  // default: gray
      cv::imshow("InputImage", cv_img);
      calcOrientedGradient(cv_gimg, cv_og_img);  // gray scale
    }

    cv::Mat cv_out_img;
    cv::cvtColor(cv_og_img, cv_out_img, CV_HSV2BGR);
    cv::imshow("OrientedGradient", cv_out_img);
    cv::waitKey(1);
  }
};

}  // namespace


int main(int argc, char** argv) {
  ros::init(argc, argv, "oriented_gradient");
  ros::NodeHandle handle("~");
  jsk_perception::OrientedGradientNode ognode(handle);
  ros::spin();
  return 0;
}


