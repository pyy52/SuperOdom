// Value types for LiDAR-assisted visual depth (Phase 3).
// Gate doc AGENT_PHASE3_LIDAR_VISUAL_DEPTH.md sections 5/14/24.
// Depth semantics (locked by test_depth_semantic_z_not_range):
//   depth ≡ Z_C, the camera optical-axis depth of (T_C_L · p_L) --
//   NEVER the LiDAR range ||p_L|| and never ||P_C||.
#pragma once

#include <map>
#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>

namespace super_odometry_vio {
namespace lidar_depth {

// One buffered LiDAR scan: points in the raw LIDAR frame at scan time, with
// the body pose at scan time attached so the associator can compensate to the
// image timestamp (identity pose => scan-stamp tier,
// docs/LIDAR_VISUAL_DEPTH_DESIGN.md section 3).
struct RecentLidarScan
{
    int64_t stamp_ns{0};
    Sophus::SE3d T_W_B;  // body pose at scan time (W gauge of the provider)
    std::vector<Eigen::Vector3d> points_lidar;
};

enum class RejectReason
{
    NONE = 0,
    NO_SUPPORT,
    TOO_OLD,
    BEHIND_CAMERA,
    OUTSIDE_FOV,
    DEPTH_DISCONTINUITY,
    HIGH_SPREAD,
    INVALID_NUMERIC,
    OUT_OF_RANGE
};

const char* toCString(RejectReason r);

// Gate section 14 output contract. depth_z_m is Z_C (section 5).
struct ExternalFeatureDepth
{
    int feature_id{0};
    int64_t image_stamp_ns{0};
    double depth_z_m{0.0};
    double robust_sigma_m{0.0};
    int support_count{0};
    double median_pixel_distance{0.0};
    double lidar_age_sec{0.0};
    bool valid{false};
};

// Gate section 34 config block (yaml: lidar_depth.*).
struct LidarDepthConfig
{
    bool enable{false};
    double max_age_sec{0.12};
    int max_scans{3};
    size_t max_points_per_scan{200000};
    double min_depth_m{0.3};
    double max_depth_m{30.0};
    double pixel_search_radius{4.0};
    int min_support_points{3};
    double max_depth_spread_m{0.3};
    double max_relative_depth_spread{0.10};
    double sigma_base_m{0.02};
    double sigma_range_scale{0.01};
    double sigma_spread_scale{0.5};
    int image_width{0};
    int image_height{0};
};

// Gate section 24 per-frame statistics (all rejection reasons counted).
struct FrameStats
{
    int projected_lidar_points{0};
    int dropped_invalid_points{0};
    int behind_camera_points{0};
    int out_of_fov_points{0};
    int out_of_range_points{0};
    int tracked_features{0};
    int features_with_candidates{0};
    int features_with_accepted_depth{0};
    std::map<RejectReason, int> rejects;
};

struct LidarDepthFrameResult
{
    std::vector<ExternalFeatureDepth> depths;
    std::map<int, RejectReason> rejected;  // feature_id -> reason
    FrameStats stats;
};

}  // namespace lidar_depth
}  // namespace super_odometry_vio
