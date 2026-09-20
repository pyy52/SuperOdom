#include "super_odometry_vio/fusion_2021/shadow_timeline.hpp"

#include <algorithm>
#include <cmath>

namespace super_odometry_vio {
namespace fusion_2021 {

const char* toCString(AcceptDecision d)
{
    switch (d)
    {
        case AcceptDecision::ACCEPTED: return "ACCEPTED";
        case AcceptDecision::REJECT_TOO_LATE: return "REJECT_TOO_LATE";
        case AcceptDecision::REJECT_DUPLICATE: return "REJECT_DUPLICATE";
        case AcceptDecision::REJECT_NO_REF: return "REJECT_NO_REF";
        case AcceptDecision::REJECT_NO_BRACKET: return "REJECT_NO_BRACKET";
        case AcceptDecision::REJECT_INNOVATION_TRANS: return "REJECT_INNOVATION_TRANS";
        case AcceptDecision::REJECT_INNOVATION_ROT: return "REJECT_INNOVATION_ROT";
    }
    return "UNKNOWN";
}

// Gate-review correction 3: GTSAM Pose3 tangent order is
// [rotation xyz (3), translation xyz (3)]. Locked here and enforced by
// test_pose3_noise_order_rotation_then_translation.
gtsam::SharedNoiseModel makePoseNoise(double sigma_rot_rad, double sigma_trans_m)
{
    gtsam::Vector6 sigmas;
    sigmas << sigma_rot_rad, sigma_rot_rad, sigma_rot_rad,
        sigma_trans_m, sigma_trans_m, sigma_trans_m;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

ShadowTimeline::ShadowTimeline(
    const std::shared_ptr<gtsam::PreintegrationParams>& imu_params,
    const ShadowConfig& config)
  : config_(config), imu_params_(imu_params)
{
    // Gate-review correction 4: 4B-1 uses unbounded shadow retention; no
    // hand-rolled marginalization (deleting variables != marginalizing).
    gtsam::ISAM2Params params;
    isam2_ = gtsam::ISAM2(params);
}

void ShadowTimeline::feedImu(int64_t stamp_ns, const gtsam::Vector3& acc,
                             const gtsam::Vector3& gyro)
{
    ++diag_.imu_samples;
    if (!imu_buf_.empty() && stamp_ns <= imu_buf_.back().stamp_ns)
    {
        // 2021 path: no silent dt fallback; malformed/out-of-order dropped.
        ++diag_.imu_dropped;
        return;
    }
    imu_buf_.push_back({stamp_ns, acc, gyro});

    if (!imu_initialized_)
    {
        imu_initialized_ = true;
        t0_ns_ = stamp_ns;
        // Grid origin = first IMU measurement time (exact t_k = t0 + k*dt_a).
        gtsam::Vector3 zero_velocity;
        zero_velocity.setZero();
        anchor_stamps_.push_back(t0_ns_);
        anchor_T_W_B_.push_back(gtsam::Pose3());
        anchor_v_W_.push_back(zero_velocity);
        anchor_bias_.push_back(gtsam::imuBias::ConstantBias());
        dT_imu_ref_.push_back(gtsam::Pose3());  // unused slot for interval -1
        ++diag_.anchors_created;

        // One-time gauge priors (design section 7) -- never recurring priors.
        auto noise6 = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
        graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            poseKey(0), gtsam::Pose3(), noise6);
        ++diag_.pose_priors_added;
        graph_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
            velKey(0), zero_velocity,
            gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
        ++diag_.vel_priors_added;
        graph_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
            biasKey(0), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
        ++diag_.bias_priors_added;
        values_.insert(poseKey(0), gtsam::Pose3());
        values_.insert(velKey(0), zero_velocity);
        values_.insert(biasKey(0), gtsam::imuBias::ConstantBias());
        graph_dirty_ = true;
    }

    createAnchorIfNeeded(stamp_ns);
}

void ShadowTimeline::createAnchorIfNeeded(int64_t imu_stamp_ns)
{
    const int64_t dt_a = static_cast<int64_t>(1e9 / config_.anchor_rate_hz);
    // Exact measurement-time grid: t_k = t0 + k*dt_a (not per-N-IMU-samples).
    while (imu_stamp_ns >= t0_ns_ + static_cast<int64_t>(anchor_stamps_.size()) * dt_a)
    {
        const int k_new = static_cast<int>(anchor_stamps_.size());
        anchor_stamps_.push_back(t0_ns_ + static_cast<int64_t>(k_new) * dt_a);
        anchor_T_W_B_.push_back(gtsam::Pose3());  // replaced by interval solve
        gtsam::Vector3 zero_vel;
        zero_vel.setZero();
        anchor_v_W_.push_back(zero_vel);
        anchor_bias_.push_back(anchor_bias_.back());
        dT_imu_ref_.push_back(gtsam::Pose3());
        ++diag_.anchors_created;
        closeInterval(k_new - 1);
    }
}

void ShadowTimeline::closeInterval(int k)
{
    if (k < 0 || k + 1 >= static_cast<int>(anchor_stamps_.size())) return;
    const int64_t t_i = anchor_stamps_[k];
    const int64_t t_j = anchor_stamps_[k + 1];

    gtsam::PreintegratedImuMeasurements pim(imu_params_, anchor_bias_[k]);
    bool bracketed = false;
    int64_t t_prev = t_i;
    for (const auto& s : imu_buf_)
    {
        if (s.stamp_ns <= t_i) continue;

        if (s.stamp_ns < t_j)
        {
            const double dt = static_cast<double>(s.stamp_ns - t_prev) * 1e-9;
            if (dt <= 0.0 || dt > config_.max_imu_dt_sec)
            {
                ++diag_.imu_dropped;
                t_prev = s.stamp_ns;
                continue;
            }
            pim.integrateMeasurement(s.acc, s.gyro, dt);
            t_prev = s.stamp_ns;
            bracketed = true;
        }
        else // s.stamp_ns >= t_j
        {
            // Exact boundary clamping under Zero-Order Hold (ZOH):
            // Integrate remaining sub-interval [t_prev, t_j] using this bracketing sample.
            const double dt = static_cast<double>(t_j - t_prev) * 1e-9;
            if (dt > 0.0)
            {
                if (dt > config_.max_imu_dt_sec)
                {
                    ++diag_.imu_dropped;
                }
                else
                {
                    pim.integrateMeasurement(s.acc, s.gyro, dt);
                    bracketed = true;
                }
            }
            t_prev = t_j;
            break;
        }
    }
    if (!bracketed)
    {
        // No IMU coverage for the interval: leave it IMU-unconstrained this
        // round (diagnosed); the anchor pair is only closed when covered.
        return;
    }
    ++diag_.intervals_closed;

    // Initial values for the new anchor by IMU prediction.
    const gtsam::NavState prev(anchor_T_W_B_[k], anchor_v_W_[k]);
    const gtsam::NavState predicted = pim.predict(prev, anchor_bias_[k]);

    // IMMUTABLE reference captured BEFORE any update that could include
    // source factors (gate-review correction 1: arrival-order-independent
    // innovation gate). Relative pose T_{B_k, B_{k+1}} predicted by IMU integration.
    dT_imu_ref_[k] = anchor_T_W_B_[k].between(predicted.pose());

    // IMU factor + bias random walk between the anchors.
    graph_.emplace_shared<gtsam::ImuFactor>(poseKey(k), velKey(k), poseKey(k + 1),
                                            velKey(k + 1), biasKey(k), pim);
    graph_.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        biasKey(k), biasKey(k + 1), gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-4));

    values_.insert(poseKey(k + 1), predicted.pose());
    values_.insert(velKey(k + 1), predicted.velocity());
    values_.insert(biasKey(k + 1), anchor_bias_[k]);
    anchor_T_W_B_[k + 1] = predicted.pose();
    anchor_v_W_[k + 1] = predicted.velocity();
    graph_dirty_ = true;
    last_closed_k_ = k;
    flushOptimizer();
    tryInsertLioFactors();
}

void ShadowTimeline::flushOptimizer()
{
    if (!graph_dirty_) return;
    isam2_.update(graph_, values_);
    graph_.resize(0);
    values_.clear();
    graph_dirty_ = false;
    const int n = static_cast<int>(anchor_stamps_.size());
    for (int kk = 0; kk < n; ++kk)
    {
        anchor_T_W_B_[kk] = isam2_.calculateEstimate<gtsam::Pose3>(poseKey(kk));
        anchor_v_W_[kk] = isam2_.calculateEstimate<gtsam::Vector3>(velKey(kk));
        anchor_bias_[kk] =
            isam2_.calculateEstimate<gtsam::imuBias::ConstantBias>(biasKey(kk));
    }
    // NOTE: dT_imu_ref_ is NOT refreshed from estimates -- it stays the
    // immutable IMU-only prediction captured at interval close.
}

void ShadowTimeline::feedLioPose(int64_t stamp_ns, const Sophus::SE3d& T_W_L,
                                 uint32_t lio_epoch)
{
    current_lio_epoch_ = lio_epoch;
    lio_buf_.push_back({stamp_ns, lio_epoch, T_W_L});
    while (lio_buf_.size() > 200) lio_buf_.pop_front();
    tryInsertLioFactors();
}

AcceptDecision ShadowTimeline::insertRelativeConstraint(
    const ConstraintKey& key, const gtsam::Pose3& T_Bi_Bj)
{
    const int k = key.k;
    if (k < 0 || k >= static_cast<int>(dT_imu_ref_.size()))
    {
        return AcceptDecision::REJECT_NO_REF;
    }
    if (k + 1 >= static_cast<int>(anchor_stamps_.size()))
    {
        return AcceptDecision::REJECT_NO_BRACKET;
    }

    const int64_t newest = anchor_stamps_.back();
    const int64_t t_j = anchor_stamps_[k + 1];
    const double lateness_sec = static_cast<double>(newest - t_j) * 1e-9;
    if (lateness_sec > config_.max_constraint_lateness_sec)
    {
        ++diag_.lio_rejected_too_late;
        lio_constrained_intervals_.insert(k);
        return AcceptDecision::REJECT_TOO_LATE;
    }

    // Dedup: check if constraint key was already inserted OR interval already constrained by this source
    if (inserted_constraints_.count(key) || lio_constrained_intervals_.count(k))
    {
        ++diag_.lio_rejected_duplicate;
        return AcceptDecision::REJECT_DUPLICATE;
    }

    // Innovation gate vs immutable IMU reference
    const gtsam::Pose3& ref = dT_imu_ref_[k];
    const gtsam::Vector6 e = gtsam::Pose3::Logmap(ref.between(T_Bi_Bj));
    const double rot_err = e.head<3>().norm();
    const double trans_err = e.tail<3>().norm();
    if (trans_err > config_.innovation_trans_m)
    {
        ++diag_.lio_rejected_innovation_trans;
        lio_constrained_intervals_.insert(k);
        return AcceptDecision::REJECT_INNOVATION_TRANS;
    }
    if (rot_err > config_.innovation_rot_rad)
    {
        ++diag_.lio_rejected_innovation_rot;
        lio_constrained_intervals_.insert(k);
        return AcceptDecision::REJECT_INNOVATION_ROT;
    }

    // Insert BetweenFactor into graph
    graph_.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        poseKey(k), poseKey(k + 1), T_Bi_Bj,
        makePoseNoise(config_.lio_sigma_rot_rad, config_.lio_sigma_trans_m)));
    graph_dirty_ = true;
    inserted_constraints_.insert(key);
    lio_constrained_intervals_.insert(k);
    ++diag_.lio_accepted;
    flushOptimizer();
    return AcceptDecision::ACCEPTED;
}

void ShadowTimeline::tryInsertLioFactors()
{
    if (last_closed_k_ < 0 || !T_B_L_set_) return;

    // Advance next_lio_scan_k_ past contiguous intervals already constrained
    while (next_lio_scan_k_ <= last_closed_k_ &&
           lio_constrained_intervals_.count(next_lio_scan_k_))
    {
        ++next_lio_scan_k_;
    }

    // Only consecutive intervals X_k -> X_{k+1} in round 1 (review note).
    for (int k = next_lio_scan_k_; k <= last_closed_k_; ++k)
    {
        if (lio_constrained_intervals_.count(k)) continue;
        const int64_t t_i = anchor_stamps_[k];
        const int64_t t_j = anchor_stamps_[k + 1];

        // Bracketing LIO samples in the SAME epoch.
        const LioSample* s_i = nullptr;
        const LioSample* s_j = nullptr;
        for (const auto& s : lio_buf_)
        {
            if (s.epoch != current_lio_epoch_) continue;
            if (s.stamp_ns <= t_i &&
                (!s_i || s.stamp_ns > s_i->stamp_ns))
                s_i = &s;
            if (s.stamp_ns >= t_j && !s_j) s_j = &s;
        }
        if (!s_i || !s_j)
        {
            ++diag_.lio_no_bracket;
            continue;  // try later: more samples may arrive for this interval
        }
        if ((s_j->stamp_ns - s_i->stamp_ns) * 1e-9 >
            config_.max_interpolation_gap_sec)
        {
            ++diag_.lio_no_bracket;
            continue;
        }

        // Body-frame differencing (gate-review correction 2):
        // T_W_B = T_W_L * inverse(T_B_L), then T_Bi_Bj = between(T_W_Bi, T_W_Bj).
        const Sophus::SE3d T_W_Bi = s_i->T_W_L * T_B_L_.inverse();
        const Sophus::SE3d T_W_Bj = s_j->T_W_L * T_B_L_.inverse();
        const gtsam::Pose3 T_W_Bi_g(
            gtsam::Rot3(T_W_Bi.rotationMatrix()),
            gtsam::Point3(T_W_Bi.translation()));
        const gtsam::Pose3 T_W_Bj_g(
            gtsam::Rot3(T_W_Bj.rotationMatrix()),
            gtsam::Point3(T_W_Bj.translation()));
        const gtsam::Pose3 T_Bi_Bj = T_W_Bi_g.between(T_W_Bj_g);

        const ConstraintKey key{0 /* SOURCE_LIO */, current_lio_epoch_, k};
        insertRelativeConstraint(key, T_Bi_Bj);
    }

    while (next_lio_scan_k_ <= last_closed_k_ &&
           lio_constrained_intervals_.count(next_lio_scan_k_))
    {
        ++next_lio_scan_k_;
    }
}

bool ShadowTimeline::latestAnchor(gtsam::Pose3& T_W_B, gtsam::Vector3& v_W,
                                  gtsam::imuBias::ConstantBias& bias) const
{
    if (anchor_T_W_B_.empty()) return false;
    const int k = static_cast<int>(anchor_T_W_B_.size()) - 1;
    T_W_B = anchor_T_W_B_[k];
    v_W = anchor_v_W_[k];
    bias = anchor_bias_[k];
    return true;
}

PredictedState ShadowTimeline::propagateTo(int64_t stamp_ns)
{
    PredictedState out;
    if (anchor_stamps_.empty()) return out;
    out.stamp_ns = stamp_ns;
    const int k = static_cast<int>(anchor_stamps_.size()) - 1;
    gtsam::PreintegratedImuMeasurements pim(imu_params_, anchor_bias_[k]);
    int64_t t_prev = anchor_stamps_[k];
    for (const auto& s : imu_buf_)
    {
        if (s.stamp_ns <= t_prev) continue;
        if (s.stamp_ns < stamp_ns)
        {
            const double dt = static_cast<double>(s.stamp_ns - t_prev) * 1e-9;
            if (dt <= 0.0 || dt > config_.max_imu_dt_sec)
            {
                t_prev = s.stamp_ns;
                continue;
            }
            pim.integrateMeasurement(s.acc, s.gyro, dt);
            t_prev = s.stamp_ns;
        }
        else // s.stamp_ns >= stamp_ns
        {
            const double dt = static_cast<double>(stamp_ns - t_prev) * 1e-9;
            if (dt > 0.0 && dt <= config_.max_imu_dt_sec)
            {
                pim.integrateMeasurement(s.acc, s.gyro, dt);
            }
            t_prev = stamp_ns;
            break;
        }
    }
    const gtsam::NavState prev(anchor_T_W_B_[k], anchor_v_W_[k]);
    const gtsam::NavState cur = pim.predict(prev, anchor_bias_[k]);
    out.T_W_B = cur.pose();
    out.v_W = cur.velocity();
    out.valid = true;
    return out;
}

}  // namespace fusion_2021
}  // namespace super_odometry_vio
