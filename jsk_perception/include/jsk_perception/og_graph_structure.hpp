// @file og_graph_structure.hpp
// @brief calc oriented gradient graph structure, from h-yaguchi SI2011
// @author Hiroaki Yaguchi, JSK

#ifndef JSK_PERCEPTION_OG_GRAPH_STRUCTURE_HPP_
#define JSK_PERCEPTION_OG_GRAPH_STRUCTURE_HPP_

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "jsk_perception/oriented_gradient.hpp"

namespace jsk_perception {

// constants
const int kOGKeyIntensityThresh = 32;
const int kOGKeyIntensityBlockSize = 2;
const int kLSHBlockSize = 16;
const double kLSHSearchRadius = 16.0;
const double kPairRelationThresh = 32.0;


// inline functions

// @brief calc sqrt of inner product between 2 oriented gradients
// @param g0 gradient intensity of OG0
// @param o0 orientation (in degree) of OG0
// @param g1 gradient intensity of OG1
// @param o1 orientation (in degree) of OG1
inline double sqrtip(int g0, int o0, int g1, int o1) {
  return sqrt(fabs(g0 * g1 * cos((o1 - o0) * M_PI / 180.0)));
}

// @brief calc sqrt of inner product between gradient orientation and direction vector
// @param xx diff of position.x
// @param yy diff of position.y
// @param o0 orientation (in degree) of OG0
inline double sqrtip2(double xx, double yy, int o0) {
  // convert from angle to geometry
  double th = o0 * M_PI / 180.0;
  double x0 = cos(th);
  double y0 = sin(th);

  // normalize direction vector
  double dist = sqrt(xx * xx + yy * yy);
  double x1 = xx / dist;
  double y1 = yy / dist;

  return sqrt(fabs(x0 * x1 + y0 * y1));
}

// @brief cosine distance of 2 oriented gradients
// @param o0 orientation (in degree) of OG0
// @param o1 orientation (in degree) of OG1
inline double ogdiff(int o0, int o1) {
  return cos((o1 - o0) * M_PI / 180.0);
}


// classes

// @brief class of Locality Sensitive Hash Table (very easy implementation)
class LSHPointTable{
 private:
  std::vector<std::vector<size_t> > indices_;
  size_t num_;
  int bwidth_, bheight_, bsize_;

 public:
  LSHPointTable(const std::vector<cv::Point>& points,
                int width,
                int height,
                int bs);
  void radiusSearch(const cv::Point& pt,
                    const std::vector<cv::Point>& points,
                    double rad,
                    std::vector<size_t>& idxs,
                    std::vector<double>& dists);
};  // LSHPointTable


// @brief oriented gradient graph structure, to search structure element
class OGGraphStructure {
 public:
  OGGraphStructure();
  ~OGGraphStructure();

  void setEvalLine();
  void setEvalParallel();

  void convert(cv::Mat& cvimg);
  int calcMaxOrientation(cv::Mat& ogimg);
  void calcPair(cv::Mat& ogimg);
  void checkConnectivity();
  void checkDistance();

  void drawResult(cv::Mat& resimg);


  // setter and getter
  int ogkey_intensity_thresh() {
    return ogkey_intensity_thresh_;
  }
  void ogkey_intensity_thresh(int val) {
    ogkey_intensity_thresh_ = val;
  }

  int ogkey_intensity_block_size() {
    return ogkey_intensity_block_size_;
  }
  void ogkey_intensity_block_size(int val) {
    ogkey_intensity_block_size_ = val;
  }

  int lsh_block_size() {
    return lsh_block_size_;
  }
  void lsh_block_size(int val) {
    lsh_block_size_ = val;
  }

  double lsh_search_radius() {
    return lsh_search_radius_;
  }
  void lsh_search_radius(double val) {
    lsh_search_radius_ = val;
  }

  double pair_relation_thresh() {
    return pair_relation_thresh_;
  }
  void pair_relation_thresh(double val) {
    pair_relation_thresh_ = val;
  }

  cv::Mat& oriented_gradient_image() {
    return ogimg_;
  }

 private:
  cv::Mat ogimg_;

  int ogkey_intensity_thresh_;
  int ogkey_intensity_block_size_;
  int lsh_block_size_;
  double lsh_search_radius_;
  double pair_relation_thresh_;

  std::vector<cv::Point> keypoints_;
  std::vector<std::pair<cv::Point, cv::Point> > line_pairs_;
  std::vector<std::pair<size_t, size_t> > line_pair_indices_;
  std::vector<std::pair<cv::Point, cv::Point> > parallel_pairs_;
  std::vector<std::pair<size_t, size_t> > parallel_pair_indices_;
  std::vector<double> parallel_pair_distances_;
  int max_theta_;

  void findEndPoints_(const std::vector<std::pair<size_t, size_t> >& pairs,
                      std::vector<size_t>& end_points);
};  // OGGraphStructure

// functions to eval og pair
double evalLinePair(int g0, int o0, int g1, int o1, double ip2);
double evalParallelPair(int g0, int o0, int g1, int o1, double ip2);

}  // namespace

#endif  // JSK_PERCEPTION_OG_GRAPH_STRUCTURE_HPP_
