// Camera-model tests for the camodocal core imported from VINS-Mono.
// Locks projection semantics used by the VIO front end and the
// LiDAR-depth associator (docs/FRAMES_AND_CALIBRATION.md).
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "camodocal/camera_models/PinholeCamera.h"

namespace {

camodocal::PinholeCamera makePinhole() {
  // TUM-VI-like intrinsics, zero distortion for closed-form checks.
  camodocal::PinholeCamera cam("test_cam", 752, 480,
                               /*k1*/ 0.0, /*k2*/ 0.0,
                               /*p1*/ 0.0, /*p2*/ 0.0,
                               /*fx*/ 450.0, /*fy*/ 450.0,
                               /*cx*/ 376.0, /*cy*/ 240.0);
  return cam;
}

}  // namespace

TEST(PinholeModel, SpaceToPlaneMatchesKModelNoDistortion) {
  const camodocal::PinholeCamera cam = makePinhole();
  const Eigen::Vector3d p_c(0.1, -0.05, 1.2);

  Eigen::Vector2d uv;
  cam.spaceToPlane(p_c, uv);

  EXPECT_NEAR(uv.x(), 376.0 + 450.0 * 0.1 / 1.2, 1e-6);
  EXPECT_NEAR(uv.y(), 240.0 + 450.0 * (-0.05) / 1.2, 1e-6);
}

TEST(PinholeModel, LiftProjectiveRoundTrip) {
  const camodocal::PinholeCamera cam = makePinhole();
  const Eigen::Vector2d uv_in(500.0, 300.0);

  Eigen::Vector3d ray;
  cam.liftProjective(uv_in, ray);

  Eigen::Vector2d uv_out;
  cam.spaceToPlane(ray, uv_out);
  EXPECT_TRUE(uv_out.isApprox(uv_in, 1e-4));
}

TEST(PinholeModel, DepthIsPreservedAlongRay) {
  const camodocal::PinholeCamera cam = makePinhole();
  const Eigen::Vector3d p_near(0.1, -0.05, 1.2);
  const Eigen::Vector3d p_far = p_near * 3.0;

  Eigen::Vector2d uv_near, uv_far;
  cam.spaceToPlane(p_near, uv_near);
  cam.spaceToPlane(p_far, uv_far);
  // Same direction ray must land on the same pixel regardless of depth.
  EXPECT_TRUE(uv_near.isApprox(uv_far, 1e-9));
}
