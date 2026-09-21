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
        case AcceptDecision::REJECT_SLOT_OCCUPIED: return "REJECT_SLOT_OCCUPIED";
        case AcceptDecision::REJECT_CROSS_EPOCH: return "REJECT_CROSS_EPOCH";
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
    if (!config_.parity_trace_file.empty()) {
        parity_tracer_ = std::make_shared<ParityTracer>(config_.parity_trace_file);
    }
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
        anchor_status_.push_back(AnchorStatus::GRAPH_INSERTED);
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
        if (parity_tracer_) {
            parity_tracer_->write("TIMELINE_ANCHOR_OPEN", t0_ns_ + static_cast<int64_t>(k_new) * dt_a, k_new, "IMU", 0, 
                "{\"t_k\":" + std::to_string(t0_ns_ + static_cast<int64_t>(k_new) * dt_a) + ",\"anchor_status\":\"GRAPH_INSERTED\",\"fusion_segment_status\":\"ACTIVE\"}");
        }
        anchor_T_W_B_.push_back(gtsam::Pose3());  // replaced by interval solve
        gtsam::Vector3 zero_vel;
        zero_vel.setZero();
        anchor_v_W_.push_back(zero_vel);
        anchor_bias_.push_back(anchor_bias_.back());
        anchor_status_.push_back(AnchorStatus::SCHEDULED);
        dT_imu_ref_.push_back(gtsam::Pose3());
        ++diag_.anchors_created;
        closeInterval(k_new - 1);
    }
}

void ShadowTimeline::closeInterval(int k)
{
    if (k < 0 || k + 1 >= static_cast<int>(anchor_stamps_.size())) return;

    if (anchor_status_[k] == AnchorStatus::INVALID_GAP)
    {
        anchor_status_[k + 1] = AnchorStatus::INVALID_GAP;
        return;
    }

    const int64_t t_i = anchor_stamps_[k];
    const int64_t t_j = anchor_stamps_[k + 1];

    gtsam::PreintegratedImuMeasurements pim(imu_params_, anchor_bias_[k]);
    int64_t covered_until_ns = t_i;
    bool gap_detected = false;
    for (const auto& s : imu_buf_)
    {
        if (s.stamp_ns <= t_i) continue;

        if (s.stamp_ns < t_j)
        {
            const int64_t dt_ns = s.stamp_ns - covered_until_ns;
            if (dt_ns <= 0 || dt_ns > config_.max_imu_dt_ns)
            {
                ++diag_.imu_dropped;
                gap_detected = true;
                covered_until_ns = s.stamp_ns;
                continue;
            }
            pim.integrateMeasurement(s.acc, s.gyro, static_cast<double>(dt_ns) * 1e-9);
            covered_until_ns = s.stamp_ns;
        }
        else // s.stamp_ns >= t_j
        {
            const int64_t dt_ns = t_j - covered_until_ns;
            if (dt_ns > 0)
            {
                if (dt_ns > config_.max_imu_dt_ns)
                {
                    ++diag_.imu_dropped;
                    gap_detected = true;
                }
                else
                {
                    pim.integrateMeasurement(s.acc, s.gyro, static_cast<double>(dt_ns) * 1e-9);
                    covered_until_ns = t_j;
                }
            }
            else if (dt_ns == 0)
            {
                // Exact sample landed on t_j
                covered_until_ns = t_j;
            }
            break;
        }
    }
    if (gap_detected || covered_until_ns != t_j)
    {
        if (gap_detected)
        {
            anchor_status_[k + 1] = AnchorStatus::INVALID_GAP;
        }
        return;
    }
    ++diag_.intervals_closed;
    if (parity_tracer_) {
        parity_tracer_->write("TIMELINE_ANCHOR_CLOSE", anchor_stamps_[k], k, "IMU", 0, 
            "{\"t_k\":" + std::to_string(anchor_stamps_[k]) + ",\"anchor_status\":\"GRAPH_INSERTED\",\"fusion_segment_status\":\"ACTIVE\"}");
    }

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
    anchor_status_[k + 1] = AnchorStatus::GRAPH_INSERTED;
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
    if (parity_tracer_) {
        for (size_t k = 0; k < anchor_stamps_.size(); ++k) {
            if (anchor_status_[k] == AnchorStatus::SOLVED) {
                gtsam::Pose3 T = anchor_T_W_B_[k];
                gtsam::Vector3 v = anchor_v_W_[k];
                gtsam::imuBias::ConstantBias b = anchor_bias_[k];
                parity_tracer_->write("STATE_SNAPSHOT", anchor_stamps_[k], k, "SYSTEM", 0, 
                    "{\"T_W_B\":" + ParityTracer::poseToJson(T) + ",\"v_W\":" + ParityTracer::vec3ToJson(v) + ",\"bias_acc\":" + ParityTracer::vec3ToJson(b.accelerometer()) + ",\"bias_gyro\":" + ParityTracer::vec3ToJson(b.gyroscope()) + ",\"anchor_status\":\"SOLVED\"}");
            }
        }
    }
    const int n = static_cast<int>(anchor_stamps_.size());
    for (int kk = 0; kk < n; ++kk)
    {
        if (anchor_status_[kk] == AnchorStatus::GRAPH_INSERTED ||
            anchor_status_[kk] == AnchorStatus::SOLVED)
        {
            anchor_T_W_B_[kk] = isam2_.calculateEstimate<gtsam::Pose3>(poseKey(kk));
            anchor_v_W_[kk] = isam2_.calculateEstimate<gtsam::Vector3>(velKey(kk));
            anchor_bias_[kk] =
                isam2_.calculateEstimate<gtsam::imuBias::ConstantBias>(biasKey(kk));
            anchor_status_[kk] = AnchorStatus::SOLVED;
        }
    }
    // NOTE: dT_imu_ref_ is NOT refreshed from estimates -- it stays the
    // immutable IMU-only prediction captured at interval close.
}

void ShadowTimeline::feedLioPose(int64_t stamp_ns, const Sophus::SE3d& T_W_L,
                                 uint32_t lio_epoch)
{
    if (stamp_ns > max_seen_event_stamp_ns_)
    {
        max_seen_event_stamp_ns_ = stamp_ns;
        watermark_ns_ = max_seen_event_stamp_ns_ - config_.source_reorder_horizon_ns;
    }

    if (stamp_ns <= watermark_ns_)
    {
        ++diag_.lio_late_after_watermark;
        if (parity_tracer_) {
            parity_tracer_->write("INPUT_CONSUME", stamp_ns, -1, "LIO", lio_epoch, 
                "{\"sensor_type\":\"LIO\",\"sample_time_ns\":" + std::to_string(stamp_ns) + ",\"action\":\"DROPPED_WATERMARK\",\"reason\":\"late_after_watermark\",\"epoch_changed\":false}");
        }
        return;
    }

    if (lio_epoch < current_lio_epoch_)
    {
        ++diag_.lio_stale_skipped;
        if (parity_tracer_) {
            parity_tracer_->write("INPUT_CONSUME", stamp_ns, -1, "LIO", lio_epoch, 
                "{\"sensor_type\":\"LIO\",\"sample_time_ns\":" + std::to_string(stamp_ns) + ",\"action\":\"DROPPED_WATERMARK\",\"reason\":\"stale_skipped\",\"epoch_changed\":false}");
        }
        return;
    }
    if (lio_epoch > current_lio_epoch_)
    {
        current_lio_epoch_ = lio_epoch;
    }
    lio_buf_.push_back({stamp_ns, lio_epoch, T_W_L});
    while (lio_buf_.size() > 200) lio_buf_.pop_front();
    tryInsertLioFactors();
}

AcceptDecision ShadowTimeline::insertRelativeConstraint(
    const ConstraintId& key, const gtsam::Pose3& T_Bi_Bj)
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
    const int64_t lateness_ns = newest - t_j;
    ConstraintSlot slot{key.source, k};
    
    if (lateness_ns > config_.max_constraint_lateness_ns)
    {
        ++diag_.lio_rejected_too_late;
        if (parity_tracer_) {
            parity_tracer_->write("GATE_EVALUATION", t_j, k, "LIO", key.epoch, 
                "{\"dT_imu_ref\":0,\"dT_source\":0,\"innovation_6d\":[0,0,0,0,0,0],\"translation_norm\":0,\"rotation_norm\":0,\"decision\":\"REJECTED\",\"reason\":\"TOO_LATE\",\"lateness_ns\":" + std::to_string(lateness_ns) + ",\"watermark_ns\":" + std::to_string(watermark_ns_) + ",\"interpolation_brackets\":[0,0]}");
        }
        finalized_slots_.insert(slot);
        return AcceptDecision::REJECT_TOO_LATE;
    }

    if (inserted_constraints_.count(key))
    {
        ++diag_.lio_rejected_duplicate;
        return AcceptDecision::REJECT_DUPLICATE;
    }

    if (committed_slots_.count(slot))
    {
        ++diag_.lio_rejected_slot_occupied;
        return AcceptDecision::REJECT_SLOT_OCCUPIED;
    }

    const gtsam::Pose3& ref = dT_imu_ref_[k];
    const gtsam::Vector6 e = gtsam::Pose3::Logmap(ref.between(T_Bi_Bj));
    const double rot_err = e.head<3>().norm();
    const double trans_err = e.tail<3>().norm();
    if (trans_err > config_.innovation_trans_m)
    {
        ++diag_.lio_rejected_innovation_trans;
        if (parity_tracer_) {
            parity_tracer_->write("GATE_EVALUATION", t_j, k, "LIO", key.epoch, 
                "{\"dT_imu_ref\":0,\"dT_source\":0,\"innovation_6d\":[0,0,0,0,0,0],\"translation_norm\":" + std::to_string(trans_err) + ",\"rotation_norm\":" + std::to_string(rot_err) + ",\"decision\":\"REJECTED\",\"reason\":\"INNOVATION_TOO_LARGE_TRANS\",\"lateness_ns\":" + std::to_string(lateness_ns) + ",\"watermark_ns\":" + std::to_string(watermark_ns_) + ",\"interpolation_brackets\":[0,0]}");
        }
        finalized_slots_.insert(slot);
        return AcceptDecision::REJECT_INNOVATION_TRANS;
    }
    if (rot_err > config_.innovation_rot_rad)
    {
        ++diag_.lio_rejected_innovation_rot;
        if (parity_tracer_) {
            parity_tracer_->write("GATE_EVALUATION", t_j, k, "LIO", key.epoch, 
                "{\"dT_imu_ref\":0,\"dT_source\":0,\"innovation_6d\":[0,0,0,0,0,0],\"translation_norm\":" + std::to_string(trans_err) + ",\"rotation_norm\":" + std::to_string(rot_err) + ",\"decision\":\"REJECTED\",\"reason\":\"INNOVATION_TOO_LARGE_ROT\",\"lateness_ns\":" + std::to_string(lateness_ns) + ",\"watermark_ns\":" + std::to_string(watermark_ns_) + ",\"interpolation_brackets\":[0,0]}");
        }
        finalized_slots_.insert(slot);
        return AcceptDecision::REJECT_INNOVATION_ROT;
    }

    graph_.push_back(gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        poseKey(k), poseKey(k + 1), T_Bi_Bj,
        makePoseNoise(config_.lio_sigma_rot_rad, config_.lio_sigma_trans_m)));
    graph_dirty_ = true;
    
    inserted_constraints_.insert(key);
    committed_slots_.insert(slot);
    finalized_slots_.insert(slot);
    committed_measurements_[key] = T_Bi_Bj;
    
    ++diag_.lio_accepted;
    if (parity_tracer_) {
        parity_tracer_->write("GATE_EVALUATION", t_j, k, "LIO", key.epoch, 
            "{\"dT_imu_ref\":0,\"dT_source\":0,\"innovation_6d\":[0,0,0,0,0,0],\"translation_norm\":" + std::to_string(trans_err) + ",\"rotation_norm\":" + std::to_string(rot_err) + ",\"decision\":\"ACCEPTED\",\"reason\":\"SUCCESS\",\"lateness_ns\":" + std::to_string(lateness_ns) + ",\"watermark_ns\":" + std::to_string(watermark_ns_) + ",\"interpolation_brackets\":[0,0]}");
        parity_tracer_->write("FACTOR_INSERT", t_j, k, "LIO", key.epoch, 
            "{\"factor_type\":\"BetweenFactorPose3\",\"keys\":[\"X_" + std::to_string(k) + "\",\"X_" + std::to_string(k+1) + "\"],\"insertion_time_ns\":" + std::to_string(t_j) + ",\"status\":\"COMMITTED\"}");
    }
    flushOptimizer();
    return AcceptDecision::ACCEPTED;
}

AcceptDecision ShadowTimeline::lookupLioPoseAt(int64_t stamp_ns, Sophus::SE3d& T_W_L, uint32_t& epoch_out) const
{
    const LioSample* s_exact = nullptr;
    const LioSample* s_before = nullptr;
    const LioSample* s_after = nullptr;

    for (const auto& s : lio_buf_)
    {
        if (s.stamp_ns == stamp_ns)
        {
            s_exact = &s;
            break;
        }
        if (s.stamp_ns < stamp_ns)
        {
            if (!s_before || s.stamp_ns > s_before->stamp_ns) s_before = &s;
        }
        else // s.stamp_ns > stamp_ns
        {
            if (!s_after || s.stamp_ns < s_after->stamp_ns) s_after = &s;
        }
    }

    if (s_exact)
    {
        T_W_L = s_exact->T_W_L;
        epoch_out = s_exact->epoch;
        return AcceptDecision::ACCEPTED;
    }

    if (!s_before || !s_after)
    {
        return AcceptDecision::REJECT_NO_BRACKET;
    }

    if (s_before->epoch != s_after->epoch)
    {
        return AcceptDecision::REJECT_CROSS_EPOCH;
    }

    const int64_t gap_ns = s_after->stamp_ns - s_before->stamp_ns;
    if (gap_ns <= 0 || gap_ns > config_.max_interpolation_gap_ns)
    {
        return AcceptDecision::REJECT_NO_BRACKET;
    }

    const double alpha =
        static_cast<double>(stamp_ns - s_before->stamp_ns) /
        static_cast<double>(gap_ns);

    const Eigen::Vector3d trans =
        (1.0 - alpha) * s_before->T_W_L.translation() +
        alpha * s_after->T_W_L.translation();

    const Eigen::Quaterniond q_before(s_before->T_W_L.unit_quaternion());
    const Eigen::Quaterniond q_after(s_after->T_W_L.unit_quaternion());
    const Eigen::Quaterniond q_interp = q_before.slerp(alpha, q_after);

    T_W_L = Sophus::SE3d(q_interp, trans);
    epoch_out = s_after->epoch;
    return AcceptDecision::ACCEPTED;
}

void ShadowTimeline::tryInsertLioFactors()
{
    if (last_closed_k_ < 0 || !T_B_L_set_) return;

    while (next_lio_scan_k_ <= last_closed_k_ &&
           finalized_slots_.count({0, next_lio_scan_k_}))
    {
        ++next_lio_scan_k_;
    }

    for (int k = next_lio_scan_k_; k <= last_closed_k_; ++k)
    {
        if (finalized_slots_.count({0, k})) continue;
        const int64_t t_i = anchor_stamps_[k];
        const int64_t t_j = anchor_stamps_[k + 1];

        // Event-time finalization with influence horizon constraint:
        if (watermark_ns_ < t_j + config_.max_interpolation_gap_ns)
        {
            break;
        }

        uint32_t epoch_i = 0;
        uint32_t epoch_j = 0;
        Sophus::SE3d T_W_L_i, T_W_L_j;
        
        AcceptDecision dec_i = lookupLioPoseAt(t_i, T_W_L_i, epoch_i);
        AcceptDecision dec_j = lookupLioPoseAt(t_j, T_W_L_j, epoch_j);
        
        if (dec_i == AcceptDecision::REJECT_CROSS_EPOCH || 
            dec_j == AcceptDecision::REJECT_CROSS_EPOCH ||
            (dec_i == AcceptDecision::ACCEPTED && dec_j == AcceptDecision::ACCEPTED && epoch_i != epoch_j))
        {
            ++diag_.lio_rejected_cross_epoch;
            finalized_slots_.insert({0, k});
            continue;
        }

        if (dec_i != AcceptDecision::ACCEPTED || dec_j != AcceptDecision::ACCEPTED)
        {
            ++diag_.lio_no_bracket;
            finalized_slots_.insert({0, k});
            continue;
        }

        const Sophus::SE3d T_W_Bi = T_W_L_i * T_B_L_.inverse();
        const Sophus::SE3d T_W_Bj = T_W_L_j * T_B_L_.inverse();
        const gtsam::Pose3 T_W_Bi_g(
            gtsam::Rot3(T_W_Bi.rotationMatrix()),
            gtsam::Point3(T_W_Bi.translation()));
        const gtsam::Pose3 T_W_Bj_g(
            gtsam::Rot3(T_W_Bj.rotationMatrix()),
            gtsam::Point3(T_W_Bj.translation()));
        const gtsam::Pose3 T_Bi_Bj = T_W_Bi_g.between(T_W_Bj_g);

        const ConstraintId key{0 /* SOURCE_LIO */, epoch_j, k};
        AcceptDecision dec = insertRelativeConstraint(key, T_Bi_Bj);
        
        // If not accepted but NOT duplicate, it's rejected terminal (since too_late, slots, innovations are finalized).
        // The slot is already marked finalized inside insertRelativeConstraint if it was a terminal failure.
    }

    while (next_lio_scan_k_ <= last_closed_k_ &&
           finalized_slots_.count({0, next_lio_scan_k_}))
    {
        ++next_lio_scan_k_;
    }
}

bool ShadowTimeline::latestAnchor(gtsam::Pose3& T_W_B, gtsam::Vector3& v_W,
                                  gtsam::imuBias::ConstantBias& bias) const
{
    if (anchor_T_W_B_.empty()) return false;
    for (int k = static_cast<int>(anchor_T_W_B_.size()) - 1; k >= 0; --k)
    {
        if (anchor_status_[k] == AnchorStatus::SOLVED)
        {
            T_W_B = anchor_T_W_B_[k];
            v_W = anchor_v_W_[k];
            bias = anchor_bias_[k];
            return true;
        }
    }
    return false;
}

PredictedState ShadowTimeline::propagateTo(int64_t stamp_ns)
{
    PredictedState out;
    if (anchor_stamps_.empty()) return out;
    out.stamp_ns = stamp_ns;

    // Select latest SOLVED anchor k such that anchor_stamps_[k] <= stamp_ns
    int k = -1;
    for (int i = static_cast<int>(anchor_stamps_.size()) - 1; i >= 0; --i)
    {
        if (anchor_stamps_[i] <= stamp_ns && anchor_status_[i] == AnchorStatus::SOLVED)
        {
            k = i;
            break;
        }
    }

    if (k < 0)
    {
        // No solved anchor before or at stamp_ns
        return out;
    }

    gtsam::PreintegratedImuMeasurements pim(imu_params_, anchor_bias_[k]);
    int64_t t_prev = anchor_stamps_[k];
    for (const auto& s : imu_buf_)
    {
        if (s.stamp_ns <= t_prev) continue;
        if (s.stamp_ns < stamp_ns)
        {
            const int64_t dt_ns = s.stamp_ns - t_prev;
            if (dt_ns <= 0 || dt_ns > config_.max_imu_dt_ns)
            {
                out.valid = false;
                return out;
            }
            pim.integrateMeasurement(s.acc, s.gyro, static_cast<double>(dt_ns) * 1e-9);
            t_prev = s.stamp_ns;
        }
        else // s.stamp_ns >= stamp_ns
        {
            const int64_t dt_ns = stamp_ns - t_prev;
            if (dt_ns > 0)
            {
                if (dt_ns > config_.max_imu_dt_ns)
                {
                    out.valid = false;
                    return out;
                }
                pim.integrateMeasurement(s.acc, s.gyro, static_cast<double>(dt_ns) * 1e-9);
            }
            t_prev = stamp_ns;
            break;
        }
    }
    
    if (t_prev != stamp_ns)
    {
        out.valid = false;
        return out;
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
