// @file parallel_saliency.hpp
// @brief calc parallel saliency using oriented gradient
// @author Hiroaki Yaguchi, JSK

#ifndef JSK_PERCEPTION_PARALLEL_SALIENCY_HPP_
#define JSK_PERCEPTION_PARALLEL_SALIENCY_HPP_

#include "jsk_perception/oriented_gradient.hpp"

namespace jsk_perception {

double calcParallelLikelihood(cv::Mat& ogimg, cv::Point& pt, int bs);
void calcParallelSaliencyMap(cv::Mat& src, cv::Mat& dst, int bs = 16);

}  // namespace

#endif  // JSK_PERCEPTION_PARALLEL_SALIENCY_HPP_
