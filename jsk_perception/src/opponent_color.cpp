// @brief calc opponent color
// @author Hiroaki Yaguchi, JSK
#include "jsk_perception/opponent_color.hpp"

namespace jsk_perception {

void cvtOpponentColor(cv::Mat& src, cv::Mat& dst) {
  int width = src.cols;
  int height = src.rows;

  dst.create(height, width, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      cv::Vec3b& px_bgr = src.at<cv::Vec3b>(y, x);
      cv::Vec3b& px_opp = dst.at<cv::Vec3b>(y, x);
      RGBtoOpponent(px_bgr, px_opp);
    }
  }
}

void RGBtoOpponent(cv::Vec3b& bgr, cv::Vec3b& opp) {
  int r = static_cast<int>(bgr[2]);
  int g = static_cast<int>(bgr[1]);
  int b = static_cast<int>(bgr[0]);

  int o1 = ((r - g) + 255) / 2;
  int o2 = ((r + g - (b * 2)) + 511) / 4;
  int o3 = (r + g + b) / 3;

  opp[0] = static_cast<unsigned char>(o1);
  opp[1] = static_cast<unsigned char>(o2);
  opp[2] = static_cast<unsigned char>(o3);
}

}  // namespace
