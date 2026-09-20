// LiDAR-depth association tests (Phase 3 gate section 22 core set).
// All geometry uses non-identity extrinsics unless stated otherwise.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <boost/make_shared.hpp>

#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>

#include "feature_manager.h"
#include "super_odometry_vio/lidar_depth/lidar_depth_associator.hpp"
#include "super_odometry_vio/vio_estimator.hpp"

namespace {

using namespace super_odometry_vio;
using namespace super_odometry_vio::lidar_depth;

camodocal::CameraPtr makePinhole()
{
    return boost::make_shared<camodocal::PinholeCamera>(
        "test_cam", 640, 480, 0.0, 0.0, 0.0, 0.0, 350.0, 350.0, 320.0, 240.0);
}

camodocal::CameraPtr makeEquidistant()
{
    return boost::make_shared<camodocal::EquidistantCamera>(
        "fisheye_cam", 512, 512, 0.0034823894, 0.00071503484, -0.0020532361,
        0.00020293673, 190.97847715, 190.97330705, 254.93170606, 256.89744290);
}

Eigen::Isometry3d makeIso(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = t;
    return T;
}

Eigen::Matrix3d rpy(double r, double p, double y)
{
    return (Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()))
        .toRotationMatrix();
}

// Standard non-identity rig used by most tests.
struct Rig
{
    Sophus::SE3d T_B_L;
    Sophus::SE3d T_B_C;
    Sophus::SE3d T_C_L;

    Rig()
        : T_B_L(Sophus::SE3d(Eigen::Quaterniond(rpy(0.1, 0.05, -0.08)).normalized(),
                             Eigen::Vector3d(0.02, -0.03, 0.04))),
          T_B_C(Sophus::SE3d(Eigen::Quaterniond(rpy(-0.12, 0.2, 0.3)).normalized(),
                             Eigen::Vector3d(0.05, 0.01, -0.02)))
    {
        T_C_L = T_B_C.inverse() * T_B_L;
    }
};

RecentLidarScan makeScan(int64_t stamp_ns, const Rig& rig,
                         const std::vector<Eigen::Vector3d>& points_C)
{
    // Caller provides points in the CAMERA frame for convenience; convert to
    // the raw LIDAR frame so the associator reconstructs p_C exactly:
    // p_L = T_B_L^-1 * T_B_C * p_C  =>  p_B = T_B_L p_L = T_B_C p_C.
    RecentLidarScan scan;
    scan.stamp_ns = stamp_ns;
    const Sophus::SE3d T_C_L = rig.T_B_L.inverse() * rig.T_B_C;
    for (const auto& p_c : points_C)
        scan.points_lidar.push_back(T_C_L * p_c);
    return scan;
}

TrackedFeatureObservation makeFeature(int id, double u, double v)
{
    TrackedFeatureObservation f;
    f.feature_id = id;
    f.pixel << u, v;
    f.bearing << 0.0, 0.0, 1.0;
    return f;
}

LidarDepthConfig baseConfig()
{
    LidarDepthConfig c;
    c.enable = true;
    c.image_width = 640;
    c.image_height = 480;
    return c;
}

namespace {
// Self-check helper: Z_C of a camera-frame point is its z coordinate.
double result_depth_ok(const Eigen::Vector3d& p_c) { return p_c.z(); }
}

// Projects a camera-frame point and returns the pixel via camodocal.
Eigen::Vector2d projectC(const camodocal::CameraPtr& cam, const Eigen::Vector3d& p_c)
{
    Eigen::Vector2d uv;
    cam->spaceToPlane(p_c, uv);
    return uv;
}

}  // namespace

TEST(DepthSemantics, DepthIsCameraOpticalAxisNotRange)
{
    // Point at 45 degrees off-axis: Z_C = 1, range = ||P_C|| = sqrt(2) here.
    Rig rig;
    LidarDepthConfig cfg = baseConfig();
    cfg.min_support_points = 1;
    LidarDepthAssociator assoc(makePinhole(), cfg, rig.T_B_L, rig.T_B_C);
    const Eigen::Vector3d p_C(0.5, 0.4, 1.0);  // projects inside the image
    const Eigen::Vector2d uv = projectC(makePinhole(), p_C);
    EXPECT_NEAR(result_depth_ok(p_C), 1.0, 1e-12);  // self-check helper below
    assoc.pushScan(makeScan(0, rig, {p_C}));

    const auto result = assoc.associate({makeFeature(7, uv.x(), uv.y())}, 100);
    ASSERT_EQ(result.depths.size(), 1u);
    EXPECT_NEAR(result.depths[0].depth_z_m, 1.0, 1e-9);      // Z_C = 1
    EXPECT_NE(result.depths[0].depth_z_m, p_C.norm());       // range = ~0.94
    EXPECT_DOUBLE_EQ(result.depths[0].image_stamp_ns, 100);
    EXPECT_NEAR(result.depths[0].lidar_age_sec, 1e-7, 1e-12);  // stamp 100ns vs scan 0
}

TEST(DepthSemantics, ExtrinsicCompositionMatchesGateFormula)
{
    const Rig rig;
    EXPECT_TRUE(rig.T_C_L.matrix().isApprox(
        (rig.T_B_C.inverse() * rig.T_B_L).matrix(), 1e-12));
}

TEST(LidarProjection, NonIdentityExtrinsicMatchesDirectCamodocal)
{
    const Rig rig;
    const auto cam = makePinhole();
    LidarDepthConfig cfg = baseConfig();
    cfg.min_support_points = 1;
    LidarDepthAssociator assoc(cam, cfg, rig.T_B_L, rig.T_B_C);

    const Eigen::Vector3d p_C(0.3, -0.2, 4.0);
    const Eigen::Vector2d uv_expected = projectC(cam, p_C);
    assoc.pushScan(makeScan(0, rig, {p_C}));

    const auto result = assoc.associate({makeFeature(1, uv_expected.x(), uv_expected.y())}, 0);
    ASSERT_EQ(result.depths.size(), 1u);
    EXPECT_NEAR(result.depths[0].depth_z_m, 4.0, 1e-9);
    EXPECT_NEAR(result.depths[0].median_pixel_distance, 0.0, 1e-9);
}

TEST(LidarProjection, WorksWithFisheyeEquidistantModel)
{
    const Rig rig;
    const auto cam = makeEquidistant();
    LidarDepthConfig cfg = baseConfig();
    cfg.min_support_points = 1;
    LidarDepthAssociator assoc(cam, cfg, rig.T_B_L, rig.T_B_C);

    const Eigen::Vector3d p_C(0.05, 0.02, 3.0);
    const Eigen::Vector2d uv = projectC(cam, p_C);
    assoc.pushScan(makeScan(0, rig, {p_C}));
    const auto result = assoc.associate({makeFeature(3, uv.x(), uv.y())}, 0);
    ASSERT_EQ(result.depths.size(), 1u);
    EXPECT_NEAR(result.depths[0].depth_z_m, 3.0, 1e-9);
}

TEST(DepthAssociation, SingleSurfaceReturnsSurfaceDepth)
{
    const Rig rig;
    const auto cam = makePinhole();
    LidarDepthAssociator assoc(cam, baseConfig(), rig.T_B_L, rig.T_B_C);

    std::vector<Eigen::Vector3d> pts;
    for (int i = -3; i <= 3; ++i)
        for (int j = -3; j <= 3; ++j)
            pts.emplace_back(0.001 * i, 0.001 * j, 5.0);
    assoc.pushScan(makeScan(0, rig, pts));

    const Eigen::Vector2d uv = projectC(cam, Eigen::Vector3d(0, 0, 5.0));
    const auto result = assoc.associate({makeFeature(9, uv.x(), uv.y())}, 0);
    ASSERT_EQ(result.depths.size(), 1u);
    EXPECT_NEAR(result.depths[0].depth_z_m, 5.0, 1e-9);
    EXPECT_EQ(result.depths[0].support_count, 49);
    // sigma = base + range term (MAD = 0 on a constant-depth surface)
    EXPECT_NEAR(result.depths[0].robust_sigma_m, 0.02 + 0.01 * 5.0, 1e-9);
}

TEST(DepthAssociation, RobustMedianRejectsSparseOutliers)
{
    const Rig rig;
    const auto cam = makePinhole();
    LidarDepthAssociator assoc(cam, baseConfig(), rig.T_B_L, rig.T_B_C);

    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i < 6; ++i)  // majority surface at 5 m (sub-pixel spacing)
        pts.emplace_back(0.0005 * i, 0.0, 5.0 + 0.01 * i);
    pts.emplace_back(0.0, 0.0, 0.8);   // outlier foreground
    pts.emplace_back(0.0, 0.0, 12.0);  // outlier background
    assoc.pushScan(makeScan(0, rig, pts));

    const Eigen::Vector2d uv = projectC(cam, Eigen::Vector3d(0, 0, 5.0));
    const auto result = assoc.associate({makeFeature(2, uv.x(), uv.y())}, 0);
    ASSERT_EQ(result.depths.size(), 1u);
    EXPECT_NEAR(result.depths[0].depth_z_m, 5.025, 0.01);  // median of 5.00..5.05
}

TEST(DepthAssociation, BimodalMixtureIsRejected)
{
    const Rig rig;
    const auto cam = makePinhole();
    LidarDepthAssociator assoc(cam, baseConfig(), rig.T_B_L, rig.T_B_C);

    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i < 3; ++i) pts.emplace_back(0.0005 * i, 0.0, 1.0);
    for (int i = 0; i < 3; ++i) pts.emplace_back(0.0005 * i, 0.0005, 9.0);
    assoc.pushScan(makeScan(0, rig, pts));

    const Eigen::Vector2d uv = projectC(cam, Eigen::Vector3d(0, 0, 5.0));
    const auto result = assoc.associate({makeFeature(4, uv.x(), uv.y())}, 0);
    ASSERT_EQ(result.depths.size(), 0u);
    ASSERT_EQ(result.rejected.count(4), 1u);
    EXPECT_EQ(result.rejected.at(4), RejectReason::DEPTH_DISCONTINUITY);
}

TEST(DepthAssociation, InsufficientSupportIsRejected)
{
    const Rig rig;
    const auto cam = makePinhole();
    LidarDepthConfig cfg = baseConfig();
    cfg.min_support_points = 3;
    LidarDepthAssociator assoc(cam, cfg, rig.T_B_L, rig.T_B_C);

    assoc.pushScan(makeScan(0, rig, {Eigen::Vector3d(0.1, 0.0, 3.0),
                                     Eigen::Vector3d(-0.1, 0.0, 3.0)}));
    const Eigen::Vector2d uv = projectC(cam, Eigen::Vector3d(0, 0, 3.0));
    const auto result = assoc.associate({makeFeature(5, uv.x(), uv.y())}, 0);
    ASSERT_EQ(result.depths.size(), 0u);
    EXPECT_EQ(result.rejected.at(5), RejectReason::NO_SUPPORT);
}

TEST(DepthAssociation, NonFinitePointsAreDroppedNotFatal)
{
    const Rig rig;
    const auto cam = makePinhole();
    LidarDepthConfig cfg = baseConfig();
    cfg.min_support_points = 1;
    LidarDepthAssociator assoc(cam, cfg, rig.T_B_L, rig.T_B_C);

    std::vector<Eigen::Vector3d> pts;
    pts.emplace_back(0.0, 0.0, 3.0);
    Eigen::Vector3d bad(0.0, 0.0, std::nan(""));
    pts.push_back(bad);
    assoc.pushScan(makeScan(0, rig, pts));
    const Eigen::Vector2d uv = projectC(cam, Eigen::Vector3d(0, 0, 3.0));
    const auto result = assoc.associate({makeFeature(6, uv.x(), uv.y())}, 0);
    ASSERT_EQ(result.depths.size(), 1u);
    EXPECT_EQ(result.stats.dropped_invalid_points, 1);
}

TEST(DepthAssociation, BehindCameraAndOutOfRangeFiltered)
{
    const Rig rig;
    const auto cam = makePinhole();
    LidarDepthConfig cfg = baseConfig();
    cfg.min_depth_m = 0.5;
    cfg.max_depth_m = 10.0;
    cfg.min_support_points = 1;
    LidarDepthAssociator assoc(cam, cfg, rig.T_B_L, rig.T_B_C);

    std::vector<Eigen::Vector3d> pts;
    pts.emplace_back(0.0, 0.0, 3.0);    // good
    pts.emplace_back(0.0, 0.0, -2.0);   // behind camera
    pts.emplace_back(0.0, 0.0, 50.0);   // out of range
    assoc.pushScan(makeScan(0, rig, pts));
    const Eigen::Vector2d uv = projectC(cam, Eigen::Vector3d(0, 0, 3.0));
    const auto result = assoc.associate({makeFeature(8, uv.x(), uv.y())}, 0);
    ASSERT_EQ(result.depths.size(), 1u);
    EXPECT_EQ(result.stats.behind_camera_points, 1);
    EXPECT_EQ(result.stats.out_of_range_points, 1);
}

TEST(DepthAssociation, ScanAgeGateExcludesStaleScans)
{
    const Rig rig;
    const auto cam = makePinhole();
    LidarDepthConfig cfg = baseConfig();
    cfg.max_age_sec = 0.05;
    LidarDepthAssociator assoc(cam, cfg, rig.T_B_L, rig.T_B_C);

    assoc.pushScan(makeScan(0, rig, {Eigen::Vector3d(0, 0, 3.0)}));
    // Query 200 ms after the scan: everything is TOO_OLD.
    const Eigen::Vector2d uv = projectC(cam, Eigen::Vector3d(0, 0, 3.0));
    const auto result = assoc.associate({makeFeature(11, uv.x(), uv.y())}, 200000000);
    EXPECT_EQ(result.depths.size(), 0u);
    EXPECT_EQ(result.stats.rejects.at(RejectReason::TOO_OLD), 1);
}

TEST(DepthSeeding, InitializesRepairsAndNeverOverwritesMatureDepth)
{
    // Synthetic FeatureManager with three tracked features (gate 17A/17B/17C).
    Eigen::Matrix3d Rs[11];
    for (auto& R : Rs) R.setIdentity();
    FeatureManager f_manager(Rs);

    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> image;
    auto addObs = [&image](int id, double x, double y, double dx)
    {
        Eigen::Matrix<double, 7, 1> p;
        p << x, y, 1.0, 320.0 + 450.0 * x, 240.0 + 450.0 * y, dx, 0.0;
        image[id].emplace_back(0, p);
    };
    // Two frames of the same three features (parallax via dx shift).
    addObs(101, -0.10, 0.02, 0.0);
    addObs(102, 0.05, -0.05, 0.0);
    addObs(103, 0.11, 0.03, 0.0);
    EXPECT_TRUE(f_manager.addFeatureCheckParallax(0, image, 0.0));

    image.clear();
    addObs(101, -0.08, 0.02, 0.0);
    addObs(102, 0.07, -0.05, 0.0);
    addObs(103, 0.14, 0.03, 0.0);
    f_manager.addFeatureCheckParallax(1, image, 0.0);

    std::vector<lidar_depth::ExternalFeatureDepth> depths;
    lidar_depth::ExternalFeatureDepth d101;
    d101.feature_id = 101; d101.depth_z_m = 3.0; d101.valid = true; d101.support_count = 5;
    depths.push_back(d101);                                   // 17A: untriangulated seed
    lidar_depth::ExternalFeatureDepth d102;
    d102.feature_id = 102; d102.depth_z_m = -1.0; d102.valid = true;  // invalid input: ignored
    depths.push_back(d102);
    lidar_depth::ExternalFeatureDepth d103;
    d103.feature_id = 103; d103.depth_z_m = 4.0; d103.valid = true; depths.push_back(d103);
    lidar_depth::ExternalFeatureDepth d999;
    d999.feature_id = 999; d999.depth_z_m = 1.0; d999.valid = true; depths.push_back(d999);

    // Pre-mark 103 as a mature triangulated feature.
    for (auto& it : f_manager.feature)
        if (it.feature_id == 103) it.estimated_depth = 7.5;

    // Frame 1 is the newest frame (start_frame == current_frame) for all
    // three features, matching the first-observation seeding policy.
    const auto stats = VioEstimator::seedFeatureManagerDepths(f_manager, depths, 0);
    EXPECT_EQ(stats.initialized, 1);   // 101 seeded (102 invalid input skipped)
    EXPECT_EQ(stats.mature_skipped, 1);  // 103 untouched
    EXPECT_EQ(stats.not_found, 1);       // 999 not in window

    for (const auto& it : f_manager.feature)
    {
        if (it.feature_id == 101) EXPECT_NEAR(it.estimated_depth, 3.0, 1e-12);
        if (it.feature_id == 103) EXPECT_NEAR(it.estimated_depth, 7.5, 1e-12);
    }
}
