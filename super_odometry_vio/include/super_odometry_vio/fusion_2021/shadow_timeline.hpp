// Phase 4B-1 shadow central backend (2021 fusion timeline, LIO-only round).
// Design: docs/FUSION_TIMELINE_DESIGN.md + AGENT_PHASE4A review corrections:
//  * anchors on the exact measurement-time grid t_k = t0 + k/anchor_rate_hz,
//    created by the IMU stream (never by source callbacks);
//  * graph_retention_mode: unbounded_shadow (no hand-rolled marginalization);
//  * every anchor interval stores an immutable IMU-only relative prediction
//    dT_imu_ref(k,k+1), captured BEFORE any optimization update touches the
//    interval, so the LIO innovation gate is independent of callback
//    arrival order (no LIO->VIO vs VIO->LIO asymmetry is even possible yet);
//  * LIO T_W_L poses are normalized to body poses through T_B_L BEFORE
//    differencing (lever-arm correctness);
//  * GTSAM Pose3 tangent noise order is [rotation(3), translation(3)] --
//    makePoseNoise() locks this and the unit test enforces it;
//  * source constraints are one-shot immutable; dedup by
//    hash(source, epoch, key_i, key_j); late factors accepted only while
//    lateness <= bound; initial round only supports consecutive intervals
//    X_k -> X_{k+1} (no long-span factors);
//  * initial gauge: one-time PriorFactors (identity pose, zero velocity,
//    zero bias) -- never recurring source priors;
//  * 2021 path has NO silent dt fallback: malformed/out-of-order IMU stamps
//    are dropped and counted.
#pragma once

#include <deque>
#include <map>
#include <string>
#include <vector>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <sophus/se3.hpp>

namespace super_odometry_vio {
namespace fusion_2021 {

struct ShadowConfig
{
    double anchor_rate_hz{10.0};
    double max_constraint_lateness_sec{0.5};
    std::string graph_retention_mode{"unbounded_shadow"};
    double max_imu_dt_sec{0.05};         // malformed interval drop bound
    double max_interpolation_gap_sec{0.15};
    // LIO relative-factor nominal noise (Pose3 tangent: rot first, then trans)
    double lio_sigma_rot_rad{0.02};
    double lio_sigma_trans_m{0.05};
    double innovation_trans_m{1.0};      // gate thresholds (separate!)
    double innovation_rot_rad{0.35};
};

enum class AcceptDecision
{
    ACCEPTED,
    REJECT_TOO_LATE,
    REJECT_DUPLICATE,
    REJECT_NO_REF,
    REJECT_NO_BRACKET,
    REJECT_INNOVATION_TRANS,
    REJECT_INNOVATION_ROT
};
const char* toCString(AcceptDecision d);

// noise model with locked Pose3 tangent order [rotation(3), translation(3)]
gtsam::SharedNoiseModel makePoseNoise(double sigma_rot_rad, double sigma_trans_m);

struct ShadowDiag
{
    int anchors_created{0};
    int imu_samples{0};
    int imu_dropped{0};
    int intervals_closed{0};
    int lio_accepted{0};
    int lio_rejected_too_late{0};
    int lio_rejected_duplicate{0};
    int lio_rejected_innovation_trans{0};
    int lio_rejected_innovation_rot{0};
    int lio_no_bracket{0};
    int lio_stale_skipped{0};
};

// High-rate predicted body state at an arbitrary measurement time.
struct PredictedState
{
    gtsam::Pose3 T_W_B;
    gtsam::Vector3 v_W;
    int64_t stamp_ns{0};
    bool valid{false};
};

class ShadowTimeline
{
  public:
    ShadowTimeline(const std::shared_ptr<gtsam::PreintegrationParams>& imu_params,
                   const ShadowConfig& config);

    void setT_B_L(const Sophus::SE3d& T_B_L)
    {
        T_B_L_ = T_B_L;
        T_B_L_set_ = true;
    }

    // IMU stream: buffers samples and creates anchors on the exact grid.
    void feedImu(int64_t stamp_ns, const gtsam::Vector3& acc,
                 const gtsam::Vector3& gyro);

    // LIO source pose (T_W_L, LiDAR frame in the LIO world gauge).
    // Converted internally to body poses via T_B_L before differencing.
    void feedLioPose(int64_t stamp_ns, const Sophus::SE3d& T_W_L,
                     uint32_t lio_epoch = 0);

    // Latest optimized anchor state + high-rate prediction to stamp_ns.
    bool latestAnchor(gtsam::Pose3& T_W_B, gtsam::Vector3& v_W,
                      gtsam::imuBias::ConstantBias& bias) const;
    PredictedState propagateTo(int64_t stamp_ns);

    const ShadowDiag& diag() const { return diag_; }
    int anchorCount() const { return static_cast<int>(anchor_stamps_.size()); }

  private:
    using Key = gtsam::Key;
    static Key poseKey(int k) { return gtsam::Symbol('x', k); }
    static Key velKey(int k) { return gtsam::Symbol('v', k); }
    static Key biasKey(int k) { return gtsam::Symbol('b', k); }

    void createAnchorIfNeeded(int64_t imu_stamp_ns);
    void closeInterval(int k);   // preintegrate IMU, add factors, store ref
    void flushOptimizer();       // isam2_.update + refresh anchor states
    void tryInsertLioFactors();

    struct ImuSample
    {
        int64_t stamp_ns;
        gtsam::Vector3 acc, gyro;
    };
    struct LioSample
    {
        int64_t stamp_ns;
        uint32_t epoch;
        Sophus::SE3d T_W_L;
    };

    ShadowConfig config_;
    std::shared_ptr<gtsam::PreintegrationParams> imu_params_;
    ShadowDiag diag_;

    std::deque<ImuSample> imu_buf_;  // time-ordered; out-of-order dropped
    bool imu_initialized_{false};
    int64_t t0_ns_{0};               // grid origin = first IMU stamp
    int last_closed_k_{-1};          // last interval fully closed
    int next_lio_scan_k_{0};         // first interval not yet tried for LIO

    // anchor grid
    std::vector<int64_t> anchor_stamps_;
    std::vector<gtsam::Pose3> anchor_T_W_B_;
    std::vector<gtsam::Vector3> anchor_v_W_;
    std::vector<gtsam::imuBias::ConstantBias> anchor_bias_;
    // IMMUTABLE IMU-only relative prediction per interval k (k -> k+1),
    // captured from the preintegrated measurements BEFORE any optimization
    // that could include source factors for this interval.
    std::vector<gtsam::Pose3> dT_imu_ref_;

    // one-shot immutable LIO constraints already inserted: interval k -> id
    std::map<int, uint64_t> inserted_lio_;

    std::deque<LioSample> lio_buf_;
    uint32_t current_lio_epoch_{0};

    Sophus::SE3d T_B_L_ = Sophus::SE3d();  // identity default (test rigs)
    bool T_B_L_set_{true};

    gtsam::ISAM2 isam2_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values values_;
    bool graph_dirty_{false};
};

}  // namespace fusion_2021
}  // namespace super_odometry_vio
