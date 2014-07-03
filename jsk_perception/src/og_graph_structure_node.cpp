// @brief sample node to use oriented gradient graph structure
// @author Hiroaki Yaguchi, JSK

#include <vector>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "jsk_perception/og_graph_structure.hpp"

namespace jsk_perception {

class OGGraphStructureNode {
 public:
  explicit OGGraphStructureNode(const ros::NodeHandle& nh) :
      handle_(nh), image_transport_(nh) {
    // read params
    std::string param_name = "ogkey_intensity_thresh";
    int ogkey_intensity_thresh = kOGKeyIntensityThresh;
    handle_.getParam(param_name, ogkey_intensity_thresh);
    oggs_.ogkey_intensity_thresh(ogkey_intensity_thresh);

    param_name = "ogkey_intensity_block_size";
    int ogkey_intensity_block_size = kOGKeyIntensityBlockSize;
    handle_.getParam(param_name, ogkey_intensity_block_size);
    oggs_.ogkey_intensity_block_size(ogkey_intensity_block_size);

    param_name = "lsh_block_size";
    int lsh_block_size = kLSHBlockSize;
    handle_.getParam(param_name, lsh_block_size);
    oggs_.lsh_block_size(lsh_block_size);

    param_name = "lsh_search_radius";
    double lsh_search_radius = kLSHSearchRadius;
    handle_.getParam(param_name, lsh_search_radius);
    oggs_.lsh_search_radius(lsh_search_radius);

    param_name = "pair_relation_thresh";
    double pair_relation_thresh = kPairRelationThresh;
    handle_.getParam(param_name, pair_relation_thresh);
    oggs_.pair_relation_thresh(pair_relation_thresh);

    x0_ = -1;
    y0_ = -1;
    x1_ = -1;
    y1_ = -1;

    image_sub_ = image_transport_.subscribe(
        "/camera/rgb/image_raw", 1,
        &OGGraphStructureNode::imageCallback, this);

    cv::namedWindow("InputImage");
    cv::namedWindow("OrientedGradientImage");
    cv::namedWindow("OGGraphStructure");

    cv::setMouseCallback("InputImage",
                         &OGGraphStructureNode::mouseHandler,
                         this);
  }

  ~OGGraphStructureNode() {
  }

  void setROI0(int x0, int y0) {
    x0_ = x0;
    y0_ = y0;
    x1_ = -1;
    y1_ = -1;
  }
  void setROI1(int x1, int y1) {
    int x0 = x0_;
    int y0 = y0_;

    if (x0 < x1) {
      x0_ = x0;
      x1_ = x1;
    } else {
      x0_ = x1;
      x1_ = x0;
    }

    if (y0 < y1) {
      y0_ = y0;
      y1_ = y1;
    } else {
      y0_ = y1;
      y1_ = y0;
    }

    if ((x1_ - x0_) < 32 || (y1_ - y0_) < 32) {
      x0_ = -1;
      x1_ = -1;
      y0_ = -1;
      y1_ = -1;
    }
  }

  static void mouseHandler(int event, int x, int y, int flags, void* param) {
    OGGraphStructureNode* self = static_cast<OGGraphStructureNode*>(param);

    switch (event) {
      case cv::EVENT_LBUTTONDOWN:
        self->setROI0(x, y);
        break;
      case cv::EVENT_LBUTTONUP:
        self->setROI1(x, y);
        break;
    }
  }

 private:
  ros::NodeHandle handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  OGGraphStructure oggs_;

  int x0_, y0_, x1_, y1_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat cv_img = cv_ptr->image;
    cv::Mat cv_out_img;

    int width = cv_img.cols;
    int height = cv_img.rows;

    if ((x0_ < 0) || (y0_ < 0) || (x1_ < 0) || (y1_ < 0) ||
        (x0_ >= width) || (y0_ >= height)) {
      oggs_.convert(cv_img);
      cv_out_img = cv_ptr->image.clone();
    } else {
      if (x1_ >= width) x1_ = width - 1;
      if (y1_ >= height) y1_ = height - 1;
      cv::Rect roi(x0_, y0_, (x1_ - x0_), (y1_ - y0_));
      cv::Mat cv_roi_img = cv_img(roi);
      oggs_.convert(cv_roi_img);
      cv_out_img = cv_roi_img.clone();
    }

    oggs_.drawResult(cv_out_img);

    cv::Mat cv_og_img;
    cv::cvtColor(oggs_.oriented_gradient_image(), cv_og_img, CV_HSV2BGR);

    cv::imshow("InputImage", cv_img);
    cv::imshow("OrientedGradientImage", cv_og_img);
    cv::imshow("OGGraphStructure", cv_out_img);
    cv::waitKey(1);
  }

  // @brief Disable copy constructor
  OGGraphStructureNode(const OGGraphStructureNode&);
  // @brief Disable = operator
  OGGraphStructureNode& operator=(const OGGraphStructureNode&);
};

}  // namespace


int main(int argc, char** argv) {
  ros::init(argc, argv, "og_graph_structure");
  ros::NodeHandle handle("~");
  jsk_perception::OGGraphStructureNode oggsnode(handle);
  ros::spin();
  return 0;
}
