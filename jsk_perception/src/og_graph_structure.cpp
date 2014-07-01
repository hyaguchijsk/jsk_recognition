// @brief calc oriented gradient graph structure, from h-yaguchi SI2011
// @author Hiroaki Yaguchi, JSK

#include "jsk_perception/og_graph_structure.hpp"

namespace jsk_perception {

// @brief constructor of LSH table as xy grid
// @param points input points
// @param width width of input points
// @param height height of input points
// @param bs block size of xy grid
LSHPointTable::LSHPointTable(const std::vector<cv::Point>& points,
                             int width,
                             int height,
                             int bs) {
  num_ = points.size();

  bsize_ = bs;
  bwidth_ = width / bsize_ + 1;  // bwidth is width of grid
  bheight_ = height / bsize_ + 1;  // bheight is height of grid
  indices_.resize(bwidth_ * bheight_);
  for (size_t i = 0; i < num_; i++) {
    double x = points[i].x;
    double y = points[i].y;
    int bx = static_cast<int>(x / bsize_);
    int by = static_cast<int>(y / bsize_);
    if ((0 <= bx && bx < bwidth_) && (0 <= by && by < bheight_))
      indices_[by * bwidth_ + bx].push_back(i);
  }
}

// @brief easy implementation of ANN search using LSH
// @param pt input point
// @param points not in use
// @param rad radius to search
// @param idxs indices of found points
// @param dists distances of found points and input points
void LSHPointTable::radiusSearch(const cv::Point& pt,
                                 const std::vector<cv::Point>& points,
                                 double rad,
                                 std::vector<size_t>& idxs,
                                 std::vector<double>& dists) {
  idxs.clear();
  dists.clear();
  double x0 = pt.x;
  double y0 = pt.y;
  int bx = static_cast<int>(x0 / bsize_);
  int by = static_cast<int>(y0 / bsize_);
  if ((0 <= bx && bx < bwidth_) && (0 <= by && by < bheight_)) {
    // search in 3x3 blocks
    for (int bxx = (bx == 0 ? 0 : bx - 1);
         bxx <= (bx < (bwidth_ - 1) ? bx + 1 : bwidth_ - 1);
         bxx++) {
      for (int byy = (by == 0 ? 0 : by - 1);
           byy <= (by < (bheight_ - 1) ? by + 1 : bheight_ - 1);
           byy++) {
        int bidx = byy * bwidth_ + bxx;
        for (size_t i = 0; i < indices_[bidx].size(); i++) {
          size_t idx = indices_[bidx][i];
          double x1 = points[idx].x;
          double y1 = points[idx].y;
          double dist = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
          if (dist <= rad * rad) {
            dists.push_back(sqrt(dist));
            idxs.push_back(idx);
          }
        }
      }
    }
  }
}



OGGraphStructure::OGGraphStructure() {
  evalPair_ = evalLinePair;

  // default params
  ogkey_intensity_thresh_ = 32;
  ogkey_intensity_block_size_ = 2;
  lsh_block_size_ = 16;
  lsh_search_radius_ = 16.0;
  pair_relation_thresh_ = 32.0;
}
OGGraphStructure::~OGGraphStructure() {
}

// @brief set eval mode to line pair
void OGGraphStructure::setEvalLine() {
  evalPair_ = evalLinePair;
}
// @brief set eval mode to parallel pair
void OGGraphStructure::setEvalParallel() {
  evalPair_ = evalParallelPair;
}

// @brief calc all procedure
// @param cvimg input image
void OGGraphStructure::convert(cv::Mat& cvimg){
  // reset previous result
  pairs_.clear();

  cv::Mat ogimg;  // oriented gradient image
  calcOGKeyPoints(cvimg, ogimg, keypoints_,
                  ogkey_intensity_thresh_,
                  ogkey_intensity_block_size_);

  // calc max orientation from histogram
  int width = ogimg.cols;
  int height = ogimg.rows;
  std::vector<int> hist(180);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      cv::Vec3b& px = ogimg.at<cv::Vec3b>(y, x);
      if (px[0] < 180) {
        hist[px[0]] += px[2];
      } else {
        std::cout << "invlid theta: " << static_cast<int>(px[0])
                  << " at " << x << ", " << y << std::endl;
      }
    }
  }
  max_theta_ = 0;
  int max_m = hist[0];
  for (int i = 1; i < 180; i++) {
    if (max_m < hist[i]) {
      max_theta_ = i;
      max_m = hist[i];
    }
  }
  std::cout << "Image Orientation = " << max_theta_ << std::endl;


  // calc pair relation
  LSHPointTable lsh(keypoints_, cvimg.cols, cvimg.rows, lsh_block_size_);
  std::vector<size_t> indices;
  std::vector<double> distances;
  double distsum = 0.0;

  // check double connection
  std::vector<int> graph;
  graph.resize(keypoints_.size());
  for (int i = 0; i < keypoints_.size(); i++)
    graph[i] = -1;

  // calc pair relation for each points
  for (size_t i = 0; i < keypoints_.size(); i++) {
    cv::Point& p0 = keypoints_[i];
    cv::Vec3b& px0 = ogimg.at<cv::Vec3b>(p0.y, p0.x);
    int g0 = px0[2];
    int o0 = px0[0];

    lsh.radiusSearch(p0, keypoints_, lsh_search_radius_, indices, distances);
    size_t idx;
    double maxip = 0.0;
    double dist;
    double od;
    for (size_t j = 0; j < indices.size(); j++) {
      size_t ii = indices[j];
      if (ii == i) continue;
      cv::Point& p1 = keypoints_[ii];
      cv::Vec3b& px1 = ogimg.at<cv::Vec3b>(p1.y, p1.x);
      int g1 = px1[2];
      int o1 = px1[0];

      // inner prod of gradient orientation and direction vector
      double xx = p1.x - p0.x;
      double yy = p1.y - p0.y;
      double ip2 = sqrtip2(xx, yy, o0);

      // pair relation evaluation
      double ip = evalPair_(g0, o0, g1, o1, ip2);
      if (ip > maxip) {
        idx = indices[j];
        dist = distances[j];
        maxip = ip;
        od = ogdiff(o0, o1);
      }
    }

    // when evaluated value is over threshold,
    if (maxip > pair_relation_thresh_) {
      // and unless already connected, then create connection
      if (graph[idx] != i) {
        pairs_.push_back(
            std::pair<cv::Point, cv::Point>(
                keypoints_[i], keypoints_[idx]));
        distsum += dist;
        graph[i] = idx;
      }
    }
  }
  std::cout << "grad:  sum of total edges: " << distsum << std::endl;

  // cehck connected lines
  std::vector<int> status;
  status.resize(keypoints_.size());
  for (size_t i = 0; i < keypoints_.size(); i++) {
    status[i] = 0;
  }
  std::vector<std::vector<int> > conlines;
  int lnum = 0;

  for (size_t i = 0; i < keypoints_.size(); i++) {
    if (status[i] > 0) continue;
    int nexti = graph[i];
    status[i] = 1;
    if (nexti < 0) continue;
    lnum++;
    conlines.resize(lnum);
    conlines[lnum - 1].push_back(i);
    while (status[nexti] == 0) {
      conlines[lnum - 1].push_back(nexti);
      status[nexti] = 1;
      nexti = graph[nexti];
      if (nexti < 0) break;
    }
  }
  std::cout << "connected lines: "
            << lnum << " / " << keypoints_.size() << std::endl;

  // check double connection
  for (size_t i = 0; i < conlines.size(); i++) {
    int headi = conlines[i][0];
    int taili = conlines[i][conlines[i].size() - 1];
    for (size_t j = 0; j < conlines.size(); j++) {
      if (i == j) continue;
      int headj = conlines[j][0];
      int tailj = conlines[j][conlines[j].size() - 1];
      if (headi == headj || headi == tailj ||
          taili == headj || taili == tailj) {
        lnum--;
      }
    }
  }
  std::cout << "connected lines: "
            << lnum << " / " << keypoints_.size() << std::endl;
}

void OGGraphStructure::getResult(std::vector<double>& res) {
}

void OGGraphStructure::drawResult(cv::Mat& resimg) {
  // draw keypoints
  for (size_t i = 0; i < keypoints_.size(); i++) {
    cv::circle(resimg, keypoints_[i], 3, cv::Scalar(0, 0, 255), 1, 4);
  }
  // draw pair structures
  for(size_t i = 0; i < pairs_.size(); i++) {
    cv::line(resimg, pairs_[i].first, pairs_[i].second,
             cv::Scalar(0, 255, 255), 2, 8);
  }

  // draw max orientation
  int width = resimg.cols;
  int height = resimg.rows;
  double xx = cos((max_theta_ + 90) * M_PI / 180.0);
  double yy = sin((max_theta_ + 90) * M_PI / 180.0);
  cv::line(resimg,
           cv::Point(width / 2 + xx * height / 2,
                     height / 2 + yy * height / 2),
           cv::Point(width / 2 - xx * height / 2,
                     height / 2 - yy * height / 2),
           cv::Scalar(255, 255, 255), 2, 8);
}

// @brief evaluation function to find line pair
// @param g0 gradient intensity of OG0
// @param o0 orientation (in degree) of OG0
// @param g1 gradient intensity of OG1
// @param o1 orientation (in degree) of OG1
// @param ip2 inner product between gradient orientation and direction vector
double evalLinePair(int g0, int o0, int g1, int o1, double ip2) {
  return sqrt(g0 * g1) * (1.0 - ip2);
}

// @brief evaluation function to find parallel pair
// @param g0 gradient intensity of OG0
// @param o0 orientation (in degree) of OG0
// @param g1 gradient intensity of OG1
// @param o1 orientation (in degree) of OG1
// @param ip2 inner product between gradient orientation and direction vector
double evalParallelPair(int g0, int o0, int g1, int o1, double ip2) {
  return sqrtip(g0, o0, g1, o1) * ip2;
}


}  // namespace
