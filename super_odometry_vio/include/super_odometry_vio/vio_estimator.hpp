// VioEstimator: thin, ROS-free wrapper around the vendored VINS-Mono
// sliding-window core (Phase 2c gate doc sections 7-12, 15).
//
// Responsibilities:
//   * own the vendored Estimator instance and drive it from measurement
//     timestamps in a single worker context (the core is not thread-safe);
//   * emit VioState / VioRelativePose / VioQuality with honest metrics;
//   * run the WARMUP -> ACTIVE -> DEGRADED -> BYPASS -> RECOVERING state
//     machine (bypass keeps light tracking, never publishes constraints);
//   * expose central-prediction and external-depth HOOKS that are stored but
//     deliberately unused in Phase 2c (no central-system coupling yet).
//
// The vendored VINS core keeps its internal IMU preintegration (gate doc
// section 5); only relative pose observations ever leave this class.
#pragma once

#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "feature_manager.h"  // vendored core (public feature list for seeding)

#include "super_odometry_vio/vio_types.hpp"

namespace super_odometry_vio {

struct VioWrapperConfig
{
    // health thresholds (documented in config/vio_health_defaults.yaml)
    int min_features_active{15};      // below -> DEGRADED
    int min_features_bypass{5};       // below for bypass_frames -> BYPASS
    int bypass_consecutive_frames{3}; // frames of starvation before BYPASS
    int bypass_timeout_sec{5};        // no usable features at all -> BYPASS
    int recovery_consecutive_frames{5};
    double min_information_eigenvalue{50.0};  // H_visual soft floor
    double bypass_information_eigenvalue{5.0};
    // consecutive-frame hysteresis against ACTIVE<->DEGRADED flapping
    // (8 frames = 0.4 s at 20 Hz)
    int degraded_hysteresis_frames{8};

    // TUM format trajectory log (empty = disabled)
    std::string trajectory_csv;
};

class VioEstimator
{
  public:
    VioEstimator();

    /// Loads the VINS settings file (camera extrinsics, IMU noise, solver).
    /// Must be called before any feed* call.
    bool loadParameters(const std::string& config_file);

    /// IMU topic from the settings file (for adapter defaults).
    std::string imuTopic() const;

    void setConfig(const VioWrapperConfig& config) { config_ = config; }

    /// Feed raw IMU samples (measurement timestamps, seconds inside the
    /// sample). Cheap: buffers only; the worker drains in order.
    void feedImu(const ImuSample& sample);

    /// Drain buffered IMU samples through the core preintegration without
    /// running a window solve (keeps the propagated state current between
    /// keyframes). Call from the single worker context.
    void propagateImu();

    /// Feed one tracked feature frame (see TrackedFeatureFrame). Runs the
    /// sliding-window solve; call from the single worker context.
    void processFeatures(const TrackedFeatureFrame& frame);

    // ---- outputs ----------------------------------------------------------
    VioState latestState() const;
    /// Relative pose between the previous and newest solved keyframe.
    /// stamp_i/stamp_j are exact measurement timestamps of both endpoints.
    VioRelativePose latestRelativePose() const;
    VioQuality latestQuality() const;
    VioHealthState healthState() const;

    // ---- hooks (Phase 2c: stored, NOT consumed; no central coupling) -----
    /// Central IMU odometry pose prediction at a measurement timestamp.
    /// Reserved for the future IMU -> VIO prediction path (gate doc 12).
    void setPosePrior(int64_t stamp_ns, const Sophus::SE3d& T_W_B);
    /// Reserved for the future central velocity/bias prediction feed.
    void setStatePrediction(int64_t stamp_ns, const Eigen::Vector3d& v_W,
                            const Eigen::Vector3d& ba, const Eigen::Vector3d& bg);
    /// Optional external metric depths (LiDAR association, Phase 3).
    /// Accepted at the boundary; consumed by seedExternalDepths.
    void setExternalDepths(const std::vector<lidar_depth::ExternalFeatureDepth>& depths);

    struct DepthSeedStats
    {
        int initialized{0};      // untriangulated feature got its first depth (gate 17A)
        int repaired{0};         // invalid/negative depth reseeded (gate 17B)
        int mature_skipped{0};   // stable triangulated depth NOT overwritten (gate 17C)
        int not_found{0};        // feature id not in the window
    };

    /// Seeds/repairs VINS feature depths from external metric depths.
    /// Vendor-zero-change: writes only through the public FeatureManager
    /// `feature` list; triangulate() skips features with depth > 0, so a seed
    /// is protected from being re-triangulated away. Only touches features
    /// with estimated_depth <= 0 (never overwrites mature depth, gate 17C).
    /// current_frame: only features first observed in this frame are seeded,
    /// because VINS estimated_depth is defined along the START-frame ray
    /// (docs section 9); cross-frame repairs would use the wrong ray and are
    /// deferred until window poses are passed in.
    static DepthSeedStats seedFeatureManagerDepths(
        FeatureManager& f_manager,
        const std::vector<lidar_depth::ExternalFeatureDepth>& depths,
        int current_frame);

    /// Applies pending external depths to the estimator's feature manager.
    DepthSeedStats seedExternalDepths(
        const std::vector<lidar_depth::ExternalFeatureDepth>& depths);

  private:
    struct Impl;
    Impl* impl_;
    std::vector<lidar_depth::ExternalFeatureDepth> pending_depths_;

    mutable std::mutex output_mutex_;
    VioWrapperConfig config_;
};

}  // namespace super_odometry_vio
