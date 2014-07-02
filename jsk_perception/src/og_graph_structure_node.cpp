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
    std::string param_name = "eval_mode";
    std::string eval_mode = "line";

    param_name = "ogkey_intensity_thresh";
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


    image_sub_ = image_transport_.subscribe(
        "/camera/rgb/image_raw", 1,
        &OGGraphStructureNode::imageCallback, this);
    cv::namedWindow("OGGraphStructure");
  }

  ~OGGraphStructureNode() {
  }

 private:
  ros::NodeHandle handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  OGGraphStructure oggs_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat cv_img = cv_ptr->image;
    cv::Mat cv_out_img = cv_ptr->image.clone();

    oggs_.convert(cv_img);
    oggs_.drawResult(cv_out_img);

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
