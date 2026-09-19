#pragma once
//
// Ported from VINS-Mono feature_tracker/src/parameters.{h,cpp}
// (upstream HKUST-Aerial-Robotics/VINS-Mono @ 90dabb5e, GPLv3).
// Only change: ros::NodeHandle parameter loading replaced by a plain
// FileStorage config path so the tracker core stays ROS-independent
// (docs/THIRD_PARTY.md). Global names kept identical to upstream.
//
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;

extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

// Reads the VINS-style settings file (cv::FileStorage). The fisheye mask is
// resolved by the caller; upstream derived it from the package config folder.
void readParameters(const std::string &config_file,
                    const std::string &fisheye_mask_path = std::string());
