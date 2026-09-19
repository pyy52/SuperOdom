// End-to-end test of the ported VINS feature-tracking core on synthetic data.
// Exercises: config load -> intrinsic load -> detection -> temporal tracking
// with known pixel shift -> ID continuity -> undistorted point bookkeeping.
#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <random>
#include <set>

#include "feature_tracker.h"
#include "parameters.h"

namespace {

// Writes a minimal VINS-style settings file with a zero-distortion pinhole
// camera whose intrinsics match the synthetic scene below.
std::string writeTestConfig(const std::string &path, int width, int height) {
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  fs << "image_topic" << "/camera/image_raw";
  fs << "imu_topic" << "/imu/data";
  fs << "max_cnt" << 120;
  fs << "min_dist" << 12;
  fs << "image_width" << width;
  fs << "image_height" << height;
  fs << "freq" << 0;
  fs << "F_threshold" << 1.0;
  fs << "show_track" << 0;
  fs << "equalize" << 0;
  fs << "fisheye" << 0;

  fs << "model_type" << "PINHOLE";
  fs << "camera_name" << "test_cam";
  fs << "image_width" << width;
  fs << "image_height" << height;
  fs << "distortion_parameters"
     << "{:" << "k1" << 0.0 << "k2" << 0.0 << "p1" << 0.0 << "p2" << 0.0 << "}";
  fs << "projection_parameters"
     << "{:" << "fx" << 450.0 << "fy" << 450.0 << "cx" << width / 2.0 << "cy"
     << height / 2.0 << "}";
  fs.release();
  return path;
}

// Scatters well-separated bright blobs on a dark background: enough corners
// for goodFeaturesToTrack, with unambiguous locations to track across frames.
cv::Mat makeTexturedScene(int width, int height, unsigned seed) {
  cv::Mat img(height, width, CV_8UC1, cv::Scalar(20));
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> ux(20, width - 20);
  std::uniform_int_distribution<int> uy(20, height - 20);
  for (int i = 0; i < 80; ++i) {
    const cv::Point c(ux(rng), uy(rng));
    cv::circle(img, c, 4, cv::Scalar(230), -1);
    cv::circle(img, c + cv::Point(3, 3), 2, cv::Scalar(120), -1);
  }
  return img;
}

}  // namespace

TEST(FeatureTrackerCore, TracksBlobFieldAcrossKnownShift) {
  const int W = 640, H = 480;
  const std::string cfg = writeTestConfig("/tmp/vio_test_config.yaml", W, H);

  readParameters(cfg);
  ASSERT_EQ(CAM_NAMES.size(), 1u);
  ASSERT_EQ(COL, W);
  ASSERT_EQ(ROW, H);
  ASSERT_FALSE(PUB_THIS_FRAME);

  FeatureTracker tracker;
  tracker.readIntrinsicParameter(CAM_NAMES.front());

  const cv::Mat img0 = makeTexturedScene(W, H, 7);
  PUB_THIS_FRAME = true;
  tracker.readImage(img0.clone(), 1.0);
  ASSERT_FALSE(tracker.ids.empty());
  const size_t n_initial = tracker.ids.size();
  EXPECT_GE(n_initial, 30u);
  // First frame: every track has count 1.
  for (int c : tracker.track_cnt) EXPECT_EQ(c, 1);
  for (unsigned int i = 0; i < tracker.ids.size(); ++i)
    EXPECT_TRUE(tracker.updateID(i));

  // Shift the whole scene by (+2, +1) px: tracked points must follow.
  const cv::Mat img1 = makeTexturedScene(W, H, 7);
  const cv::Mat shift = (cv::Mat_<double>(2, 3) << 1, 0, 2, 0, 1, 1);
  cv::Mat img1_shifted;
  cv::warpAffine(img1, img1_shifted, shift, img1.size(), cv::INTER_LINEAR,
                 cv::BORDER_CONSTANT, cv::Scalar(20));

  PUB_THIS_FRAME = true;
  tracker.readImage(img1_shifted.clone(), 1.05);

  ASSERT_FALSE(tracker.cur_pts.empty());
  // Temporal tracks exist: some point survived with count 2.
  int continued = 0;
  for (int c : tracker.track_cnt)
    if (c >= 2) ++continued;
  EXPECT_GE(continued, 20);

  // The node owns ID assignment after each frame (upstream
  // feature_tracker_node.cpp): new detections carry -1 until then.
  for (unsigned int i = 0; i < tracker.ids.size(); ++i)
    EXPECT_TRUE(tracker.updateID(i));

  // Undistorted bookkeeping stays consistent with the tracked points.
  EXPECT_EQ(tracker.cur_un_pts.size(), tracker.cur_pts.size());
  const std::set<int> distinct_ids(tracker.ids.begin(), tracker.ids.end());
  EXPECT_EQ(tracker.cur_un_pts_map.size(), distinct_ids.size());
  EXPECT_FALSE(tracker.pts_velocity.empty());
}
