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
  // default params
  ogkey_intensity_thresh_ = kOGKeyIntensityThresh;
  ogkey_intensity_block_size_ = kOGKeyIntensityBlockSize;
  lsh_block_size_ = kLSHBlockSize;
  lsh_search_radius_ = kLSHSearchRadius;
  pair_relation_thresh_ = kPairRelationThresh;
}
OGGraphStructure::~OGGraphStructure() {
}


// @brief calc all procedure
// @param cvimg input image
void OGGraphStructure::convert(cv::Mat& cvimg){
  // oriented gradient key points
  calcOGKeyPoints(cvimg, ogimg_, keypoints_,
                  ogkey_intensity_thresh_,
                  ogkey_intensity_block_size_);

  // // calc max orientation from histogram
  // max_theta_ = calcMaxOrientation(ogimg_);
  // std::cout << "Image Orientation = " << max_theta_ << std::endl;

  // calc pair
  calcPair(ogimg_);

  // check pair relation
  // for line detection, check connectivity
  checkConnectivity();
  // for parallel detection, check distribution of distance of all pairs
  checkDistance();
}

int OGGraphStructure::calcMaxOrientation(cv::Mat& ogimg) {
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
  int max_theta = 0;
  int max_m = hist[0];
  for (int i = 1; i < 180; i++) {
    if (max_m < hist[i]) {
      max_theta = i;
      max_m = hist[i];
    }
  }
  return max_theta;
}

void OGGraphStructure::calcPair(cv::Mat& ogimg) {
  // reset previous result
  line_pairs_.clear();
  line_pair_indices_.clear();
  parallel_pairs_.clear();
  parallel_pair_indices_.clear();
  parallel_pair_distances_.clear();

  // create lsh table
  LSHPointTable lsh(keypoints_, ogimg.cols, ogimg.rows, lsh_block_size_);
  std::vector<size_t> indices;
  std::vector<double> distances;
  double distsum = 0.0;

  // check double connection
  std::vector<std::vector<int> > line_graph;
  line_graph.resize(keypoints_.size());
  std::vector<std::vector<int> > parallel_graph;
  parallel_graph.resize(keypoints_.size());

  // calc pair relation for each points
  for (size_t i = 0; i < keypoints_.size(); i++) {
    cv::Point& p0 = keypoints_[i];
    cv::Vec3b& px0 = ogimg.at<cv::Vec3b>(p0.y, p0.x);
    int g0 = px0[2];
    int o0 = px0[0];

    // search all nearest points in lsh_search_radius_
    lsh.radiusSearch(p0, keypoints_, lsh_search_radius_, indices, distances);
    size_t line_idx;
    double line_maxip = 0.0;
    size_t parallel_idx;
    double parallel_maxip = 0.0;
    double parallel_dist;
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
      double line_ip = evalLinePair(g0, o0, g1, o1, ip2);
      double parallel_ip = evalParallelPair(g0, o0, g1, o1, ip2);
      // in this case only MAX value pair is selected
      if (line_ip > line_maxip) {
        line_idx = indices[j];
        line_maxip = line_ip;
      }
      if (parallel_ip > parallel_maxip) {
        parallel_idx = indices[j];
        parallel_dist = distances[j];
        parallel_maxip = parallel_ip;
      }
    }

    // when evaluated value is over threshold,
    if (line_maxip > pair_relation_thresh_) {
      // and unless already connected, then create connection
      bool is_connected = false;
      for (size_t j = 0; j < line_graph[line_idx].size(); j++) {
        if (line_graph[line_idx][j] == i) {
          is_connected = true;
          break;
        }
      }
      if (!is_connected) {
        line_pairs_.push_back(
            std::pair<cv::Point, cv::Point>(
                keypoints_[i], keypoints_[line_idx]));
        line_pair_indices_.push_back(
            std::pair<size_t, size_t>(i, line_idx));
        line_graph[i].push_back(line_idx);
      }
    }

    // when evaluated value is over threshold,
    if (parallel_maxip > pair_relation_thresh_) {
      // and unless already connected, then create connection
      bool is_connected = false;
      for (size_t j = 0; j < parallel_graph[parallel_idx].size(); j++) {
        if (parallel_graph[parallel_idx][j] == i) {
          is_connected = true;
          break;
        }
      }
      if (!is_connected) {
        parallel_pairs_.push_back(
            std::pair<cv::Point, cv::Point>(
                keypoints_[i], keypoints_[parallel_idx]));
        parallel_pair_indices_.push_back(
            std::pair<size_t, size_t>(i, parallel_idx));
        parallel_pair_distances_.push_back(parallel_dist);
        parallel_graph[i].push_back(parallel_idx);
      }
    }
  }
}

void OGGraphStructure::checkConnectivity() {
  // cehck connected lines
  std::vector<int> status;
  std::vector<std::vector<int> > graph2;
  status.resize(keypoints_.size());
  graph2.resize(keypoints_.size());
  for (size_t i = 0; i < keypoints_.size(); i++) {
    status[i] = 0;
  }
  for (size_t i = 0; i < line_pair_indices_.size(); i++) {
    graph2[line_pair_indices_[i].first].push_back(
        line_pair_indices_[i].second);
  }

  std::vector<size_t> end_points;
  findEndPoints_(line_pair_indices_, end_points);

  std::vector<std::vector<int> > conlines;
  int lnum = 0;

  for (size_t i = 0; i < keypoints_.size(); i++) {
    if (status[i] > 0) continue;
    int nexti = graph2[i].empty() ? -1 : graph2[i][0];
    status[i] = 1;
    if (nexti < 0) continue;
    lnum++;
    conlines.resize(lnum);
    conlines[lnum - 1].push_back(i);
    while (status[nexti] == 0) {
      conlines[lnum - 1].push_back(nexti);
      status[nexti] = 1;
      nexti = graph2[i].empty() ? -1 : graph2[i][0];
      if (nexti < 0) break;
    }
  }
  // std::cout << "connected lines: "
  //           << lnum << " / " << keypoints_.size() << std::endl;
  std::cout << "connected lines: "
            << lnum << " / "
            << line_pairs_.size() << " / "
            << keypoints_.size()<< std::endl;

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
  // std::cout << "connected lines: "
  //           << lnum << " / " << keypoints_.size() << std::endl;
  std::cout << "connected lines: "
            << lnum << " / "
            << line_pairs_.size() << " / "
            << keypoints_.size()<< std::endl;
}

void OGGraphStructure::checkDistance() {
  double mean = 0.0;
  double variance = 0.0;
  for (size_t i = 0; i < parallel_pair_distances_.size(); i++) {
    mean += parallel_pair_distances_[i];
  }
  mean /= parallel_pair_distances_.size();
  for (size_t i = 0; i < parallel_pair_distances_.size(); i++) {
    double diff = parallel_pair_distances_[i] - mean;
    variance += (diff * diff);
  }
  variance /= parallel_pair_distances_.size();
  std::cout << "distance distribution:" << std::endl;
  std::cout << "  mean: " << mean << std::endl;
  std::cout << "  variance: " << variance << std::endl;

  // create another keypoints
  std::vector<cv::Point> parallel_keypoints;
  for (size_t i = 0; i < parallel_pair_distances_.size(); i++) {
    if ((parallel_pair_distances_[i] > (mean - 1.0)) &&
        (parallel_pair_distances_[i] < (mean + 1.0))) {
      parallel_keypoints.push_back(
          cv::Point((parallel_pairs_[i].first.x +
                     parallel_pairs_[i].second.x) /2,
                    (parallel_pairs_[i].first.y +
                     parallel_pairs_[i].second.y) /2));
    }
  }
  std::cout << "parallel keypoints: "
            << parallel_keypoints.size()
            << " / " << parallel_pairs_.size() << std::endl;
}

void OGGraphStructure::drawResult(cv::Mat& resimg) {
  // draw keypoints
  for (size_t i = 0; i < keypoints_.size(); i++) {
    cv::circle(resimg, keypoints_[i], 3, cv::Scalar(0, 0, 255), 1, 4);
  }
  // draw line pair structures
  for(size_t i = 0; i < line_pairs_.size(); i++) {
    cv::line(resimg, line_pairs_[i].first, line_pairs_[i].second,
             cv::Scalar(196, 0, 0), 2, 8);
  }

  // draw parallel pair structures
  for(size_t i = 0; i < parallel_pairs_.size(); i++) {
    cv::line(resimg, parallel_pairs_[i].first, parallel_pairs_[i].second,
             cv::Scalar(0, 196, 0), 1, 8);
  }

  // // draw max orientation
  // int width = resimg.cols;
  // int height = resimg.rows;
  // double xx = cos((max_theta_ + 90) * M_PI / 180.0);
  // double yy = sin((max_theta_ + 90) * M_PI / 180.0);
  // cv::line(resimg,
  //          cv::Point(width / 2 + xx * height / 2,
  //                    height / 2 + yy * height / 2),
  //          cv::Point(width / 2 - xx * height / 2,
  //                    height / 2 - yy * height / 2),
  //          cv::Scalar(255, 255, 255), 2, 8);
}

void OGGraphStructure::findEndPoints_(
    const std::vector<std::pair<size_t, size_t> >& pairs,
    std::vector<size_t>& end_points) {
  std::vector<int> vote_table;
  vote_table.resize(keypoints_.size());
  for (size_t i = 0; i < vote_table.size(); i++) {
    vote_table[i] = 0;
  }

  for (size_t i = 0; i < pairs.size(); i++) {
    vote_table[pairs[i].first] += 1;
    vote_table[pairs[i].second] += 1;
  }

  end_points.clear();
  for (size_t i = 0; i < vote_table.size(); i++) {
    if (vote_table[i] == 1) {
      end_points.push_back(i);
    }
  }
  std::cout << " found " << end_points.size() << " end point(s)" << std::endl;
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
