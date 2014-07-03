// @brief calc parallel saliency
// @author Hiroaki Yaguchi, JSK
#include "jsk_perception/parallel_saliency.hpp"

namespace jsk_perception {

double calcParallelLikelihood(cv::Mat& ogimg, cv::Point& pt, int bs) {
  int width = ogimg.cols;
  int height = ogimg.rows;
  double value = 0.0;

  int x0 = (pt.x - bs);
  int y0 = (pt.y - bs);
  int x1 = (pt.x + bs);
  int y1 = (pt.y + bs);

  // when outside of area, then return 0
  if (x0 < 0 || y0 < 0 ||
      x1 >= width || y1 >= height) {
    return 0.0;
  }

  // when region includes no interest points, then return 0
  double sum_mag = 0.0;
  for (int x = x0; x < x1; x++) {
    for (int y = y0; y < y1; y++) {
      cv::Vec3d& px = ogimg.at<cv::Vec3d>(y, x);
      sum_mag += px[2];
    }
  }
  sum_mag /= ((bs * bs) * 4);
  if (sum_mag < 12.0) {
    return 0.0;
  }


  for (int i = 0; i < bs; i++) {
    int xx0 = pt.x - i;
    int xx1 = pt.x + i;
    for (int j = 0; j < bs; j++) {
      if ((i == 0) && (j == 0)) continue;
      int yy0 = pt.y - j;
      int yy1 = pt.y + j;

      cv::Vec3d& px0 = ogimg.at<cv::Vec3d>(yy0, xx0);
      cv::Vec3d& px1 = ogimg.at<cv::Vec3d>(yy1, xx1);

      double dist = sqrt(static_cast<double>((i * i) + (j * j)));

      if (dist < 3.0 || dist > static_cast<double>(bs))
        continue;

      double cos_o = static_cast<double>(i) / dist;
      double sin_o = static_cast<double>(j) / dist;

      double c0 = px0[0];
      double s0 = px0[1];
      double v0 = px0[2];
      double c1 = px1[0];
      double s1 = px1[1];
      double v1 = px1[2];

      // cosine distance
      double cosd0 = sqrt(fabs(c0 * cos_o + s0 * sin_o));
      double cosd1 = sqrt(fabs(c1 * cos_o + s1 * sin_o));
      double cosd = cosd0 * cosd1 * v0 * v1;
      value += sqrt(cosd);
    }
  }

  // normalize
  value /= ((bs * bs) * 4);
  return value;
}

void calcParallelSaliencyMap(cv::Mat& ogimg, cv::Mat& dst, int bs) {
  int width = ogimg.cols;
  int height = ogimg.rows;

  // preprocessing sin and cos
  cv::Mat ogcsimg;
  ogcsimg.create(height, width, CV_64FC3);
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      cv::Vec2d& og = ogimg.at<cv::Vec2d>(y, x);
      double rad = og[0] / 180.0 * M_PI;
      ogcsimg.at<cv::Vec3d>(y, x) = cv::Vec3d(cos(rad), sin(rad), og[1]);
    }
  }

  dst.create(height, width, CV_8UC3);
  double max_val = 0.0;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      cv::Point pt(x, y);
      double val = calcParallelLikelihood(ogcsimg, pt, bs);
      if (val > max_val) max_val = val;

      // scale
      val *= 8;
      if (val > 255.0) val = 255.0;

      cv::Vec3b& px = dst.at<cv::Vec3b>(y, x);
      px[0] = static_cast<unsigned char>(val);
      px[1] = px[0];
      px[2] = 0;

    }
  }
  std::cout << "max: " << max_val << std::endl;
}

}  // namespace
