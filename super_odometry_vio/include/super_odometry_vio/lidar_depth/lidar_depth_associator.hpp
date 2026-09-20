// Robust LiDAR -> visual feature depth association (Phase 3).
// Projection exclusively via camodocal (gate section 7); depth is Z_C
// (section 5); image-space bucket index is per-frame and temporary
// (section 12) -- no global map, no KD-tree/octree.
#pragma once

#include <memory>
#include <unordered_map>

#include <camodocal/camera_models/Camera.h>

 #include "super_odometry_vio/vio_types.hpp"  // TrackedFeatureObservation
#include "super_odometry_vio/lidar_depth/lidar_depth_buffer.hpp"
#include "super_odometry_vio/lidar_depth/lidar_depth_types.hpp"

namespace super_odometry_vio {
namespace lidar_depth {

class LidarDepthAssociator
{
  public:
    LidarDepthAssociator(camodocal::CameraPtr camera, const LidarDepthConfig& config,
                         const Sophus::SE3d& T_B_L, const Sophus::SE3d& T_B_C);

    void pushScan(RecentLidarScan&& scan);

    // Associates the given tracked features with LiDAR support at the image
    // measurement timestamp. T_W_B_img is the body pose at image time (used
    // only to compensate buffered scans to the image time; scan-stamp tier
    // passes identity and scans carry identity, giving no compensation).
    LidarDepthFrameResult associate(
        const std::vector<TrackedFeatureObservation>& features,
        int64_t image_stamp_ns, const Sophus::SE3d& T_W_B_img = Sophus::SE3d());

    // Projected in-FOV LiDAR samples of the LAST associate() call (capped),
    // for the debug overlay (gate section 21).
    const std::vector<Eigen::Vector2f>& lastProjectedPoints() const
    {
        return last_projected_;
    }

    // T_C_L = inverse(T_B_C) * T_B_L (gate section 6), exposed for tests.
    const Sophus::SE3d& T_C_L() const { return T_C_L_; }
    const LidarDepthConfig& config() const { return config_; }

  private:
    struct Sample
    {
        float u, v;
        double z_c;
        double age_sec;
    };

    void buildIndex(int64_t image_stamp_ns, const Sophus::SE3d& T_W_B_img);

    camodocal::CameraPtr camera_;
    LidarDepthConfig config_;
    Sophus::SE3d T_B_L_, T_B_C_, T_C_L_;
    LidarDepthBuffer buffer_;

    // per-frame temporary index (cleared at the start of every associate())
    double cell_size_{4.0};
    std::unordered_map<uint64_t, std::vector<Sample>> buckets_;
    FrameStats stats_;
    std::vector<Eigen::Vector2f> last_projected_;
};

}  // namespace lidar_depth
}  // namespace super_odometry_vio
