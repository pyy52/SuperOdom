#include "super_odometry_vio/lidar_depth/lidar_depth_associator.hpp"

#include <algorithm>
#include <cmath>

namespace super_odometry_vio {
namespace lidar_depth {

const char* toCString(RejectReason r)
{
    switch (r)
    {
        case RejectReason::NONE: return "NONE";
        case RejectReason::NO_SUPPORT: return "NO_SUPPORT";
        case RejectReason::TOO_OLD: return "TOO_OLD";
        case RejectReason::BEHIND_CAMERA: return "BEHIND_CAMERA";
        case RejectReason::OUTSIDE_FOV: return "OUTSIDE_FOV";
        case RejectReason::DEPTH_DISCONTINUITY: return "DEPTH_DISCONTINUITY";
        case RejectReason::HIGH_SPREAD: return "HIGH_SPREAD";
        case RejectReason::INVALID_NUMERIC: return "INVALID_NUMERIC";
        case RejectReason::OUT_OF_RANGE: return "OUT_OF_RANGE";
    }
    return "UNKNOWN";
}

namespace {

uint64_t bucketKey(int cx, int cy)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
           static_cast<uint32_t>(cy);
}

}  // namespace

LidarDepthAssociator::LidarDepthAssociator(camodocal::CameraPtr camera,
                                           const LidarDepthConfig& config,
                                           const Sophus::SE3d& T_B_L,
                                           const Sophus::SE3d& T_B_C)
  : camera_(camera), config_(config), T_B_L_(T_B_L), T_B_C_(T_B_C),
    T_C_L_(T_B_C.inverse() * T_B_L), buffer_(config),
    cell_size_(std::max(1.0, config.pixel_search_radius))
{
}

void LidarDepthAssociator::pushScan(RecentLidarScan&& scan)
{
    buffer_.push(std::move(scan));
}

void LidarDepthAssociator::buildIndex(int64_t image_stamp_ns,
                                      const Sophus::SE3d& T_W_B_img)
{
    buckets_.clear();
    stats_ = FrameStats{};
    last_projected_.clear();

    const double max_age = config_.max_age_sec;
    // Body motion from scan time to image time, per scan (identity in both
    // frames => scan-stamp tier, no compensation).
    for (const auto& scan : buffer_.scans())
    {
        const double age_sec =
            static_cast<double>(image_stamp_ns - scan.stamp_ns) * 1e-9;
        if (age_sec < 0.0 || age_sec > max_age)
        {
            ++stats_.rejects[RejectReason::TOO_OLD];
            continue;
        }
        const Sophus::SE3d T_Bimg_Bscan = T_W_B_img.inverse() * scan.T_W_B;

        for (const auto& p_L : scan.points_lidar)
        {
            const Eigen::Vector3d p_B_scan = T_B_L_ * p_L;
            const Eigen::Vector3d p_B_img = T_Bimg_Bscan * p_B_scan;
            const Eigen::Vector3d p_C = T_B_C_.inverse() * p_B_img;

            if (!p_C.allFinite())
            {
                ++stats_.dropped_invalid_points;
                continue;
            }
            const double z = p_C.z();
            if (z <= 0.0)
            {
                ++stats_.behind_camera_points;
                continue;
            }
            if (z < config_.min_depth_m || z > config_.max_depth_m)
            {
                ++stats_.out_of_range_points;
                continue;
            }

            Eigen::Vector2d uv;
            camera_->spaceToPlane(p_C, uv);
            if (!uv.allFinite() || uv.x() < 0 || uv.y() < 0 ||
                uv.x() >= config_.image_width || uv.y() >= config_.image_height)
            {
                ++stats_.out_of_fov_points;
                continue;
            }

            const int cx = static_cast<int>(uv.x() / cell_size_);
            const int cy = static_cast<int>(uv.y() / cell_size_);
            buckets_[bucketKey(cx, cy)].push_back(
                Sample{static_cast<float>(uv.x()), static_cast<float>(uv.y()), z,
                       age_sec});
            if (last_projected_.size() < 20000)
                last_projected_.emplace_back(static_cast<float>(uv.x()),
                                             static_cast<float>(uv.y()));
            ++stats_.projected_lidar_points;
        }
    }
}

LidarDepthFrameResult LidarDepthAssociator::associate(
    const std::vector<TrackedFeatureObservation>& features,
    int64_t image_stamp_ns, const Sophus::SE3d& T_W_B_img)
{
    LidarDepthFrameResult result;
    buildIndex(image_stamp_ns, T_W_B_img);
    result.stats = stats_;

    const double r2 = config_.pixel_search_radius * config_.pixel_search_radius;
    const int span = static_cast<int>(cell_size_);

    result.stats.tracked_features = static_cast<int>(features.size());
    for (const auto& feat : features)
    {
        // Gather candidates from the 3x3 cell neighbourhood.
        const int cx = static_cast<int>(feat.pixel.x() / cell_size_);
        const int cy = static_cast<int>(feat.pixel.y() / cell_size_);
        std::vector<double> zs;
        std::vector<double> pix_dists;
        double max_age_used = 0.0;
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                const auto it = buckets_.find(bucketKey(cx + dx, cy + dy));
                if (it == buckets_.end()) continue;
                for (const Sample& s : it->second)
                {
                    const double du = s.u - feat.pixel.x();
                    const double dv = s.v - feat.pixel.y();
                    const double d2 = du * du + dv * dv;
                    if (d2 > r2) continue;
                    zs.push_back(s.z_c);
                    pix_dists.push_back(std::sqrt(d2));
                    max_age_used = std::max(max_age_used, s.age_sec);
                }
            }
        }

        if (zs.empty())
        {
            result.rejected[feat.feature_id] = RejectReason::NO_SUPPORT;
            ++result.stats.rejects[RejectReason::NO_SUPPORT];
            continue;
        }
        ++result.stats.features_with_candidates;

        // Robust median over sorted depths.
        std::sort(zs.begin(), zs.end());
        const double median =
            zs.size() % 2 == 1
                ? zs[zs.size() / 2]
                : 0.5 * (zs[zs.size() / 2 - 1] + zs[zs.size() / 2]);

        // Inner cluster around the median: foreground/background mixtures are
        // resolved by keeping only the contiguous layer near the median; a
        // bimodal mixture whose median layer is under-populated is rejected.
        std::vector<double> inner;
        for (double z : zs)
            if (std::abs(z - median) <= config_.max_depth_spread_m)
                inner.push_back(z);

        if (static_cast<int>(inner.size()) < config_.min_support_points)
        {
            const RejectReason reason = zs.size() >= static_cast<size_t>(config_.min_support_points)
                                            ? RejectReason::DEPTH_DISCONTINUITY
                                            : RejectReason::NO_SUPPORT;
            result.rejected[feat.feature_id] = reason;
            ++result.stats.rejects[reason];
            continue;
        }

        // Robust spread of the accepted layer (MAD around the layer median).
        std::sort(inner.begin(), inner.end());
        const double inner_median =
            inner.size() % 2 == 1
                ? inner[inner.size() / 2]
                : 0.5 * (inner[inner.size() / 2 - 1] + inner[inner.size() / 2]);
        std::vector<double> abs_dev(inner.size());
        for (size_t i = 0; i < inner.size(); ++i)
            abs_dev[i] = std::abs(inner[i] - inner_median);
        std::sort(abs_dev.begin(), abs_dev.end());
        const double mad = abs_dev[abs_dev.size() / 2];
        if (inner_median > 1e-9 &&
            mad > config_.max_relative_depth_spread * inner_median)
        {
            result.rejected[feat.feature_id] = RejectReason::HIGH_SPREAD;
            ++result.stats.rejects[RejectReason::HIGH_SPREAD];
            continue;
        }

        std::sort(pix_dists.begin(), pix_dists.end());
        const double median_pix =
            pix_dists.empty()
                ? 0.0
                : (pix_dists.size() % 2 == 1
                       ? pix_dists[pix_dists.size() / 2]
                       : 0.5 * (pix_dists[pix_dists.size() / 2 - 1] +
                                pix_dists[pix_dists.size() / 2]));

        ExternalFeatureDepth d;
        d.feature_id = feat.feature_id;
        d.image_stamp_ns = image_stamp_ns;
        d.depth_z_m = inner_median;
        d.robust_sigma_m =
            config_.sigma_base_m + config_.sigma_range_scale * inner_median +
            config_.sigma_spread_scale * 1.4826 * mad;
        d.support_count = static_cast<int>(inner.size());
        d.median_pixel_distance = median_pix;
        d.lidar_age_sec = max_age_used;
        d.valid = true;
        result.depths.push_back(d);
        ++result.stats.features_with_accepted_depth;
    }
    return result;
}

}  // namespace lidar_depth
}  // namespace super_odometry_vio
