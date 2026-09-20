// Bounded recent-scan cache (Phase 3 gate section 8/11/30).
// This is NOT a map: a fixed-capacity deque of recent scans with point-count
// caps, pruned causally by age. No global 3D index is ever built.
#pragma once

#include <algorithm>
#include <deque>

#include "super_odometry_vio/lidar_depth/lidar_depth_types.hpp"

namespace super_odometry_vio {
namespace lidar_depth {

class LidarDepthBuffer
{
  public:
    explicit LidarDepthBuffer(const LidarDepthConfig& config) : config_(config) {}

    void push(RecentLidarScan&& scan)
    {
        if (scan.points_lidar.size() > config_.max_points_per_scan)
            scan.points_lidar.resize(config_.max_points_per_scan);
        scans_.push_back(std::move(scan));
        while (static_cast<int>(scans_.size()) > config_.max_scans)
            scans_.pop_front();
    }

    // Causal pruning: scans older than (stamp_ns - max_age) can no longer
    // contribute to any future frame and are dropped.
    void dropOlderThan(int64_t oldest_useful_stamp_ns)
    {
        while (!scans_.empty() &&
               scans_.front().stamp_ns < oldest_useful_stamp_ns)
            scans_.pop_front();
    }

    const std::deque<RecentLidarScan>& scans() const { return scans_; }
    size_t size() const { return scans_.size(); }
    bool empty() const { return scans_.empty(); }
    void clear() { scans_.clear(); }

  private:
    LidarDepthConfig config_;
    std::deque<RecentLidarScan> scans_;
};

}  // namespace lidar_depth
}  // namespace super_odometry_vio
