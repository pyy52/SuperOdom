// Core value types for the SuperOdom 2021-parity standalone VIO
// (super_odometry_vio). These are the ROS-free interfaces between the
// vendored VINS-Mono estimator core and any adapter / future fusion layer
// (Phase 2c gate doc sections 7, 8, 15).
//
// Frame conventions follow docs/FRAMES_AND_CALIBRATION.md and
// test_vins_frames.cpp: T_W_B with body = IMU; ric = R_B_C; tic = p_B_C.
#pragma once

#include <cstdint>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "super_odometry_vio/lidar_depth/lidar_depth_types.hpp"

namespace super_odometry_vio {

struct ImuSample
{
    int64_t stamp_ns;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
};

struct TrackedFeatureObservation
{
    int feature_id;
    Eigen::Vector2d pixel;
    Eigen::Vector3d bearing;  // normalised camera-frame ray (z = 1)
    Eigen::Vector2d velocity; // pixel velocity
};

// One tracked frame: features observed at a common measurement timestamp.
struct TrackedFeatureFrame
{
    int64_t stamp_ns;
    std::vector<TrackedFeatureObservation> observations;
};

// Latest fused VIO state in the world gauge of the running estimator.
struct VioState
{
    int64_t stamp_ns;
    Sophus::SE3d T_W_B;
    Eigen::Vector3d velocity_W{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bias_accel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bias_gyro{Eigen::Vector3d::Zero()};
    bool initialized{false};
};

enum class VioHealthState
{
    WARMUP,      // estimator not yet initialized (or re-initializing)
    ACTIVE,      // sliding-window solving, constraints published
    DEGRADED,    // solving but quality below soft thresholds
    BYPASS,      // visual input unusable: keep light tracking, no constraints
    RECOVERING   // inputs look good again, restarting initialization
};

// Visual observability metrics. These are honest measurements of the visual
// geometry only (Gate doc section 10): a high feature count does NOT imply
// health. No fabricated covariance is reported anywhere in Phase 2c.
struct VioQuality
{
    int tracked_features{0};      // features in the sliding window
    int inlier_features{0};       // features with a valid observation in the newest frame
    double inlier_ratio{0.0};
    double median_parallax_px{0.0};
    // -1.0 = not computed. Phase 2c does not fabricate this value; the
    // vendored core does not expose per-frame residuals (gate doc section 9).
    double reprojection_rmse_px{-1.0};

    // visual_observability_proxy: sum(J^T J) of the newest frame's
    // projection Jacobians w.r.t. the 6-DOF relative body motion (unit
    // weights, visual-only block). Gate decision B: valid as an
    // observability proxy for Phase 2c; NOT a true marginal information --
    // ignores landmark uncertainty and window cross-correlations, and mixes
    // rotation(rad)/translation(m) scales in one Hessian. Before central
    // fusion (Phase 3/4): add robust-loss/pixel-noise/inlier weighting and
    // per-DOF normalisation; do not use as central-fusion covariance.
    double information_min_eigenvalue{0.0};
    double information_max_eigenvalue{0.0};
    double information_condition_number{0.0};
    int observable_dof{0};

    bool optimizer_success{false};
    // Sliding-window solve duration over a rolling window of frames.
    double solve_time_ms_p50{0.0};
    double solve_time_ms_p95{0.0};
    VioHealthState health{VioHealthState::WARMUP};
};

// Relative-pose constraint output for the future central fusion (section 8).
// Endpoints are measurement timestamps, required to be exact from day one.
// The central estimator must only ever consume this relative observation --
// never VINS' internal IMU/bias/marginalisation information (section 6).
struct VioRelativePose
{
    int64_t stamp_i_ns{0};
    int64_t stamp_j_ns{0};
    Sophus::SE3d T_Bi_Bj;  // maps points from body_i to body_j
    VioQuality quality;
};

// Optional external metric depth (LiDAR association, Phase 3) now lives in
// lidar_depth/lidar_depth_types.hpp with the gate section 14 field contract;
// re-exported here for the wrapper API.
using lidar_depth::ExternalFeatureDepth;

const char* toCString(VioHealthState s);

}  // namespace super_odometry_vio
