// @file opponent_color.hpp
// @brief calc opponent color space
// @author Hiroaki Yaguchi, JSK

#ifndef JSK_PERCEPTION_OPPONENT_COLOR_HPP_
#define JSK_PERCEPTION_OPPONENT_COLOR_HPP_

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace jsk_perception {

void cvtOpponentColor(cv::Mat& src, cv::Mat& dst);
void RGBtoOpponent(cv::Vec3b& rgb, cv::Vec3b& opp);

}

#endif  // JSK_PERCEPTION_OPPONENT_COLOR_HPP_

