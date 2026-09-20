#include "super_odometry_vio/vio_estimator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>

#include "estimator.h"

namespace super_odometry_vio {

const char* toCString(VioHealthState s)
{
    switch (s)
    {
        case VioHealthState::WARMUP: return "WARMUP";
        case VioHealthState::ACTIVE: return "ACTIVE";
        case VioHealthState::DEGRADED: return "DEGRADED";
        case VioHealthState::BYPASS: return "BYPASS";
        case VioHealthState::RECOVERING: return "RECOVERING";
    }
    return "UNKNOWN";
}

namespace {

constexpr double kNsToSec = 1e-9;
constexpr double kSecToNs = 1e9;

Sophus::SE3d se3FromEstimator(const Eigen::Vector3d& p, const Eigen::Matrix3d& R)
{
    return Sophus::SE3d(Eigen::Quaterniond(R).normalized(), p);
}

bool stateNotFinite(const Estimator& est)
{
    const int i = est.frame_count;
    const auto bad = [](double v) { return !std::isfinite(v); };
    for (int k = 0; k <= i; ++k)
    {
        if (est.Ps[k].unaryExpr(bad).any() || est.Rs[k].unaryExpr(bad).any() ||
            est.Vs[k].unaryExpr(bad).any())
            return true;
    }
    return est.ric[0].unaryExpr(bad).any();
}

double medianOf(std::vector<double>& v)
{
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

}  // namespace

struct VioEstimator::Impl
{
    Estimator est;
    bool parameters_loaded = false;

    // Input buffers (filled from adapter threads, drained by the worker).
    std::deque<ImuSample> imu_buf;
    std::deque<TrackedFeatureFrame> feature_buf;
    size_t max_feature_buf = 30;  // bounded, frames dropped when far behind

    // Health state machine
    VioHealthState health = VioHealthState::WARMUP;
    int starving_frames = 0;
    int degraded_frames = 0;
    int recovery_frames = 0;
    // consecutive-frame hysteresis against ACTIVE<->DEGRADED flapping
    int consecutive_degraded = 0;
    int consecutive_healthy = 0;
    double last_feature_stamp_sec = -1.0;

    // IMU drain state (upstream node semantics: dt = inter-sample diff,
    // |dt| >= 1 s treated as dropout and skipped)
    double last_imu_stamp_sec = -1.0;

    // Outputs (guarded by the wrapper output mutex)
    VioState state;
    int stale_feature_frames = 0;
    int nonfinite_feature_drops = 0;

    std::deque<double> solve_ms;
    DepthSeedStats last_seed_stats;
    void recordSolveTime(double ms)
    {
        solve_ms.push_back(ms);
        if (solve_ms.size() > 500) solve_ms.pop_front();
    }

    // Vendored processIMU takes the INTER-SAMPLE dt (see estimator.cpp), not
    // an absolute time; upstream computes it in the node from consecutive
    // stamps and skips gaps >= 1 s. Mirrored here: drain buffered samples in
    // stamp order up to max_stamp_sec, keeping newer ones buffered.
    void drainImu(double max_stamp_sec)
    {
        while (!imu_buf.empty() &&
               imu_buf.front().stamp_ns * kNsToSec <= max_stamp_sec)
        {
            const ImuSample s = imu_buf.front();
            imu_buf.pop_front();
            const double t = s.stamp_ns * kNsToSec;
            if (last_imu_stamp_sec >= 0)
            {
                const double dt = t - last_imu_stamp_sec;
                if (std::abs(dt) < 1.0)
                    est.processIMU(dt, s.accel, s.gyro);
            }
            last_imu_stamp_sec = t;
        }
    }
    VioRelativePose relative_pose;
    VioQuality quality;

    // Hooks (Phase 2c: stored, unused)
    int64_t pose_prior_stamp_ns = 0;
    Sophus::SE3d pose_prior;
    std::vector<ExternalFeatureDepth> external_depths;

void storeOutputs(const VioWrapperConfig& cfg,
                                      double frame_stamp_sec,
                                      int64_t frame_stamp_ns)
{
    (void)cfg;

    state.stamp_ns = frame_stamp_ns;
    state.T_W_B = se3FromEstimator(est.Ps[est.frame_count], est.Rs[est.frame_count]);
    state.velocity_W = est.Vs[est.frame_count];
    state.bias_accel = est.Bas[est.frame_count];
    state.bias_gyro = est.Bgs[est.frame_count];
    state.initialized = est.solver_flag == Estimator::NON_LINEAR;

    // Visual observability (gate doc section 10): H_visual = sum(J^T J) over
    // correspondences between the two newest window frames, Jacobians of the
    // normalised projection w.r.t. the 6-DOF relative body motion
    // (delta_t first, then delta_angle, expressed in frame j).
    const int i = est.frame_count - 1;
    const int j = est.frame_count;
    if (i < 0) return;

    const Eigen::Matrix3d R_WBi = est.Rs[i];
    const Eigen::Vector3d p_WBi = est.Ps[i];
    const Eigen::Matrix3d R_WBj = est.Rs[j];
    const Eigen::Vector3d p_WBj = est.Ps[j];
    const Eigen::Matrix3d R_ji = R_WBj.transpose() * R_WBi;  // R_Bi_Bj
    const Eigen::Vector3d t_ji = R_WBj.transpose() * (p_WBi - p_WBj);

    const Eigen::Matrix3d& R_B_C = est.ric[0];
    const Eigen::Vector3d& p_B_C = est.tic[0];

    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    std::vector<double> parallax;
    int correspondences = 0;

    for (const auto& feat : est.f_manager.feature)
    {
        if (feat.feature_per_frame.size() < 2) continue;
        const auto& f_i = feat.feature_per_frame[feat.feature_per_frame.size() - 2];
        const auto& f_j = feat.feature_per_frame.back();
        if (feat.solve_flag == 2) continue;  // NOTE: FeaturePerFrame::parallax is uninitialised upstream - do not read it

        const Eigen::Vector3d pc_i = f_i.point;  // normalised camera coords
        const Eigen::Vector3d pc_j = f_j.point;
        if (pc_i.z() <= 0 || pc_j.z() <= 0) continue;

        const Eigen::Vector3d pb_i = R_B_C * pc_i + p_B_C;
        const Eigen::Vector3d pb_j = R_ji * pb_i + t_ji;

        // d(q)/d(p_c) = (I - q z^T) / z for q = p_c / z
        Eigen::Matrix<double, 2, 3> dq_dp;
        dq_dp.row(0) << 1.0 / pc_j.z(), 0.0, -pc_j.x() / (pc_j.z() * pc_j.z());
        dq_dp.row(1) << 0.0, 1.0 / pc_j.z(), -pc_j.y() / (pc_j.z() * pc_j.z());

        Eigen::Matrix<double, 3, 6> dpb_dxi;
        dpb_dxi.block<3, 3>(0, 0).setIdentity();
        dpb_dxi.block<3, 3>(0, 3) = -Sophus::SO3d::hat(pb_j);

        const Eigen::Matrix<double, 2, 6> J = dq_dp * R_B_C.transpose() * dpb_dxi;
        H.noalias() += J.transpose() * J;

        parallax.push_back((f_j.uv - f_i.uv).norm());
        ++correspondences;
    }

    if (correspondences < 3)
    {
        quality.information_min_eigenvalue = 0.0;
        quality.information_max_eigenvalue = 0.0;
        quality.information_condition_number = 0.0;
        quality.observable_dof = 0;
    }
    else
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H);
        const auto& eigs = es.eigenvalues();
        quality.information_min_eigenvalue = std::max(0.0, eigs(0));
        quality.information_max_eigenvalue = std::max(0.0, eigs(5));
        quality.information_condition_number =
            quality.information_min_eigenvalue > 1e-12
                ? quality.information_max_eigenvalue /
                      quality.information_min_eigenvalue
                : std::numeric_limits<double>::infinity();
        quality.observable_dof = static_cast<int>(
            (eigs.array() > 1e-6 * std::max(1.0, quality.information_max_eigenvalue))
                .count());
    }

    quality.median_parallax_px = medianOf(parallax);

    if (!solve_ms.empty())
    {
        std::vector<double> times(solve_ms.begin(), solve_ms.end());
        quality.solve_time_ms_p50 = medianOf(times);
        std::nth_element(times.begin(), times.begin() + static_cast<long>(times.size() * 0.95),
                         times.end());
        quality.solve_time_ms_p95 = times[times.size() * 0.95];
    }
    // NOTE: reprojection RMSE is intentionally NOT reported in Phase 2c
    // (the vendored core does not expose per-frame residuals); the sentinel
    // documents that it is absent rather than fabricating a value.
    quality.reprojection_rmse_px = -1.0;
}

};

VioEstimator::VioEstimator() : impl_(new Impl()) {}

bool VioEstimator::loadParameters(const std::string& config_file)
{
    readParameters(config_file);
    impl_->est.setParameter();
    impl_->parameters_loaded = true;
    return true;
}

VioEstimator::DepthSeedStats VioEstimator::seedFeatureManagerDepths(
    FeatureManager& f_manager,
    const std::vector<lidar_depth::ExternalFeatureDepth>& depths,
    int current_frame)
{
    DepthSeedStats stats;
    for (const auto& d : depths)
    {
        if (!d.valid || d.depth_z_m <= 0.0) continue;
        bool found = false;
        for (auto& it : f_manager.feature)
        {
            if (it.feature_id != d.feature_id) continue;
            found = true;
            if (it.start_frame != current_frame)
            {
                // Not a first-observation frame: seeding the current-frame
                // Z_C onto the start-frame ray would be semantically wrong.
                // Deferred until start-frame-ray depths are computed.
                ++stats.mature_skipped;
                break;
            }
            if (it.estimated_depth > 0.0)
            {
                // Mature triangulated depth: diagnostics only, never overwrite
                // (gate 17C).
                ++stats.mature_skipped;
                break;
            }
            // estimated_depth holds metric depth along the start-frame ray;
            // vector2double exports the inverse to para_Feature and
            // triangulate() skips depth > 0, so the seed survives as the
            // optimization initial value (gate 17A).
            it.estimated_depth = d.depth_z_m;
            ++stats.initialized;
            break;
        }
        if (!found) ++stats.not_found;
    }
    return stats;
}

VioEstimator::DepthSeedStats VioEstimator::seedExternalDepths(
    const std::vector<lidar_depth::ExternalFeatureDepth>& depths)
{
    return seedFeatureManagerDepths(impl_->est.f_manager, depths,
                                    impl_->est.frame_count);
}

std::string VioEstimator::imuTopic() const
{
    return IMU_TOPIC;  // vendored global from the settings file
}

void VioEstimator::feedImu(const ImuSample& sample)
{
    Impl& p = *impl_;
    p.imu_buf.push_back(sample);
    // Bounded: drop oldest if the consumer stalls badly.
    while (p.imu_buf.size() > 4000) p.imu_buf.pop_front();
}

namespace {

// Rebuilds the vendored core's image structure:
// key = feature id, value = per-camera (7,1) [x, y, z=1, u, v, vx, vy].
std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> toVinsImage(
    const TrackedFeatureFrame& frame)
{
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> image;
    for (const auto& obs : frame.observations)
    {
        Eigen::Matrix<double, 7, 1> x;
        x << obs.bearing.x(), obs.bearing.y(), 1.0, obs.pixel.x(), obs.pixel.y(),
            obs.velocity.x(), obs.velocity.y();
        image[obs.feature_id].emplace_back(0, x);
    }
    return image;
}

}  // namespace

void VioEstimator::propagateImu()
{
    Impl& p = *impl_;
    if (!p.parameters_loaded) return;

    std::lock_guard<std::mutex> lock(output_mutex_);
    p.drainImu(std::numeric_limits<double>::infinity());

    // Keep the published propagated state current between keyframes.
    const int idx = p.est.frame_count;
    p.state.T_W_B = se3FromEstimator(p.est.Ps[idx], p.est.Rs[idx]);
    p.state.velocity_W = p.est.Vs[idx];
    p.state.bias_accel = p.est.Bas[idx];
    p.state.bias_gyro = p.est.Bgs[idx];
}

void VioEstimator::processFeatures(const TrackedFeatureFrame& frame)
{
    Impl& p = *impl_;
    if (!p.parameters_loaded) return;

    std::lock_guard<std::mutex> lock(output_mutex_);

    const double stamp_sec = frame.stamp_ns * kNsToSec;

    // 0) Boundary sanitisation: the upstream fisheye-mask profile can emit
    // non-finite bearings at the equidistant model's validity edge
    // (liftProjective fails outside the calibrated FOV). Drop those
    // observations here - no NaN may ever reach the solver (gate decision C:
    // hard failures must never linger).
    TrackedFeatureFrame clean = frame;
    clean.observations.erase(
        std::remove_if(clean.observations.begin(), clean.observations.end(),
                       [](const TrackedFeatureObservation& o)
                       {
                           return !o.bearing.allFinite() || !o.pixel.allFinite() ||
                                  !o.velocity.allFinite() || o.bearing.z() <= 0;
                       }),
        clean.observations.end());
    if (clean.observations.size() != frame.observations.size())
    {
        p.nonfinite_feature_drops +=
            static_cast<int>(frame.observations.size() - clean.observations.size());
    }

    // 1) Sync drain: integrate every IMU sample up to the frame stamp first
    // (upstream getMeasurements semantics). A frame older than already-
    // integrated IMU would corrupt the window bookkeeping and is dropped.
    if (stamp_sec < p.last_imu_stamp_sec)
    {
        ++p.stale_feature_frames;
        return;
    }
    p.drainImu(stamp_sec);

    const int n_features = static_cast<int>(clean.observations.size());

    // 2) Health state machine (gate doc section 11).
    if (p.health == VioHealthState::BYPASS)
    {
        // BYPASS: keep receiving/evaluating, never run the backend.
        if (n_features >= config_.min_features_active)
        {
            if (++p.recovery_frames >= config_.recovery_consecutive_frames)
            {
                p.est.clearState();
                p.est.setParameter();
                p.health = VioHealthState::RECOVERING;
                p.quality.health = p.health;  // publish the transition frame
                p.recovery_frames = 0;
                p.starving_frames = 0;
            }
        }
        else
        {
            p.recovery_frames = 0;
        }
        return;
    }

    if (n_features < config_.min_features_bypass &&
        ++p.starving_frames >= config_.bypass_consecutive_frames)
    {
        p.health = VioHealthState::BYPASS;
        p.quality.health = VioHealthState::BYPASS;
        return;
    }
    p.starving_frames = 0;

    // 3) Vendored core: features at measurement time, then solve.
    std_msgs::Header header;
    header.stamp.sec = stamp_sec;
    const auto solve_start = std::chrono::steady_clock::now();
    p.est.processImage(toVinsImage(clean), header);
    // Phase 3: consume external depths after this frame's triangulation so
    // seeds are in place before the next solve's vector2double().
    if (!p.external_depths.empty())
    {
        p.last_seed_stats =
            seedFeatureManagerDepths(p.est.f_manager, p.external_depths,
                                     p.est.frame_count);
        p.external_depths.clear();
    }
    p.recordSolveTime(std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - solve_start)
                          .count());

    // 4) Quality + observability metrics, then state machine update.
    p.storeOutputs(config_, stamp_sec, frame.stamp_ns);

    // NOTE: upstream's `is_valid` member is never assigned (vestigial,
    // uninitialised) and must not be read; solver_flag + failure_occur carry
    // the real state (failureDetection clears the solver back to INITIAL).
    const bool solving = p.est.solver_flag == Estimator::NON_LINEAR &&
                         !p.est.failure_occur;
    p.quality.optimizer_success = solving;
    p.quality.tracked_features = p.est.f_manager.getFeatureCount();
    p.quality.inlier_features = p.est.f_manager.last_track_num;
    p.quality.inlier_ratio =
        p.quality.tracked_features > 0
            ? static_cast<double>(p.quality.inlier_features) /
                  p.quality.tracked_features
            : 0.0;

    // Hard failures bypass all hysteresis: gate doc section C decision -
    // NaN/Inf/invalid SE(3)/solver failure must go straight to re-init,
    // never linger in DEGRADED while publishing garbage.
    if (!solving)
    {
        p.health = VioHealthState::WARMUP;
        p.consecutive_degraded = 0;
        p.consecutive_healthy = 0;
    }
    else if (stateNotFinite(p.est))
    {
        p.est.clearState();
        p.est.setParameter();
        p.last_imu_stamp_sec = -1.0;  // re-seed after restart
        p.health = VioHealthState::BYPASS;
        p.quality.health = p.health;
        return;
    }
    else if (p.quality.inlier_features < config_.min_features_active ||
             p.quality.information_min_eigenvalue <
                 config_.min_information_eigenvalue)
    {
        if (++p.consecutive_degraded >= config_.degraded_hysteresis_frames)
        {
            p.health = VioHealthState::DEGRADED;
            p.consecutive_healthy = 0;
        }
        // below bypass floor: immediate downgrade, no hysteresis
        if (p.quality.information_min_eigenvalue <
            config_.bypass_information_eigenvalue)
        {
            p.health = VioHealthState::BYPASS;
        }
    }
    else
    {
        if (++p.consecutive_healthy >= config_.degraded_hysteresis_frames ||
            p.health != VioHealthState::DEGRADED)
        {
            p.health = VioHealthState::ACTIVE;
        }
        p.consecutive_degraded = 0;
    }
    p.quality.health = p.health;

    // 5) Relative-pose constraint (gate doc section 8): previous window
    // frame -> newest solved frame, exact measurement-time endpoints.
    if (solving && p.est.frame_count >= 1)
    {
        const int idx_j = p.est.frame_count;
        const int idx_i = idx_j - 1;
        const double t_i = p.est.Headers[idx_i].stamp.toSec();
        const double t_j = p.est.Headers[idx_j].stamp.toSec();
        if (t_j > t_i)
        {
            const Sophus::SE3d T_W_Bj =
                se3FromEstimator(p.est.Ps[idx_j], p.est.Rs[idx_j]);
            const Sophus::SE3d T_W_Bi =
                se3FromEstimator(p.est.Ps[idx_i], p.est.Rs[idx_i]);
            p.relative_pose.stamp_i_ns =
                static_cast<int64_t>(t_i * kSecToNs);
            p.relative_pose.stamp_j_ns = frame.stamp_ns;
            p.relative_pose.T_Bi_Bj = T_W_Bi.inverse() * T_W_Bj;
            p.relative_pose.quality = p.quality;
        }
    }

    // 6) TUM-format trajectory log.
    if (!config_.trajectory_csv.empty())
    {
        std::ofstream fout(config_.trajectory_csv, std::ios::app);
        fout << std::fixed << std::setprecision(9) << stamp_sec << " "
             << p.est.Ps[p.est.frame_count].x() << " "
             << p.est.Ps[p.est.frame_count].y() << " "
             << p.est.Ps[p.est.frame_count].z() << " "
             << Eigen::Quaterniond(p.est.Rs[p.est.frame_count]).x() << " "
             << Eigen::Quaterniond(p.est.Rs[p.est.frame_count]).y() << " "
             << Eigen::Quaterniond(p.est.Rs[p.est.frame_count]).z() << " "
             << Eigen::Quaterniond(p.est.Rs[p.est.frame_count]).w() << "\n";
    }
}

VioState VioEstimator::latestState() const
{
    std::lock_guard<std::mutex> lock(output_mutex_);
    return impl_->state;
}

VioRelativePose VioEstimator::latestRelativePose() const
{
    std::lock_guard<std::mutex> lock(output_mutex_);
    return impl_->relative_pose;
}

VioQuality VioEstimator::latestQuality() const
{
    std::lock_guard<std::mutex> lock(output_mutex_);
    return impl_->quality;
}

VioHealthState VioEstimator::healthState() const
{
    std::lock_guard<std::mutex> lock(output_mutex_);
    return impl_->health;
}

void VioEstimator::setPosePrior(int64_t stamp_ns, const Sophus::SE3d& T_W_B)
{
    std::lock_guard<std::mutex> lock(output_mutex_);
    impl_->pose_prior_stamp_ns = stamp_ns;
    impl_->pose_prior = T_W_B;  // Phase 2c: stored, not consumed
}

void VioEstimator::setStatePrediction(int64_t stamp_ns,
                                      const Eigen::Vector3d& v_W,
                                      const Eigen::Vector3d& ba,
                                      const Eigen::Vector3d& bg)
{
    std::lock_guard<std::mutex> lock(output_mutex_);
    (void)stamp_ns;  // Phase 2c: stored, not consumed
    (void)v_W;
    (void)ba;
    (void)bg;
}

void VioEstimator::setExternalDepths(
    const std::vector<lidar_depth::ExternalFeatureDepth>& depths)
{
    std::lock_guard<std::mutex> lock(output_mutex_);
    impl_->external_depths = depths;
}

}  // namespace super_odometry_vio
