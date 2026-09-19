// Frame-convention tests locking docs/FRAMES_AND_CALIBRATION.md.
// Convention: T_X_Y maps a point expressed in frame Y into frame X:
//   p_X = T_X_Y * p_Y,   v_X = R_X_Y * v_Y
// Frames: W world, B IMU body, L LiDAR, C camera.
#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace {

Eigen::Isometry3d makeT(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}

Eigen::Matrix3d rpy(double r, double p, double y) {
  return (Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()))
      .toRotationMatrix();
}

}  // namespace

TEST(Se3Convention, CompositionAssociatesAcrossFrames) {
  const Eigen::Isometry3d T_W_B = makeT(rpy(0.1, -0.2, 0.3), Eigen::Vector3d(1.0, 2.0, 3.0));
  const Eigen::Isometry3d T_B_L = makeT(rpy(-0.05, 0.02, 1.1), Eigen::Vector3d(0.08, 0.029, 0.03));
  const Eigen::Isometry3d T_L_C = makeT(rpy(0.3, 0.1, -0.2), Eigen::Vector3d(-0.01, 0.0, 0.02));

  const Eigen::Isometry3d chain = T_W_B * T_B_L * T_L_C;
  const Eigen::Isometry3d grouped = T_W_B * (T_B_L * T_L_C);
  EXPECT_TRUE(chain.matrix().isApprox(grouped.matrix(), 1e-12));

  // Round trip: expressing a world point back from camera frame must undo exactly.
  const Eigen::Vector3d p_w(0.4, -0.7, 2.2);
  const Eigen::Isometry3d T_C_W = chain.inverse();
  EXPECT_TRUE((T_C_W * (chain * p_w)).isApprox(p_w, 1e-12));
}

TEST(Se3Convention, DoubleInverseIsIdentity) {
  const Eigen::Isometry3d T_B_L = makeT(rpy(0.7, -1.2, 0.05), Eigen::Vector3d(0.08, 0.029, 0.03));
  EXPECT_TRUE(T_B_L.inverse().inverse().matrix().isApprox(T_B_L.matrix(), 1e-12));
}

TEST(ExtrinsicConvention, Vlp16CalibMapsLaserOriginIntoImuTranslation) {
  // config/velodyne/vlp_16_calibration.yaml: R_B_L = I, t_B_L = [0.08, 0.029, 0.03].
  const Eigen::Isometry3d T_B_L =
      makeT(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.08, 0.029, 0.03));
  const Eigen::Vector3d p_imu = T_B_L * Eigen::Vector3d::Zero();
  EXPECT_TRUE(p_imu.isApprox(Eigen::Vector3d(0.08, 0.029, 0.03), 1e-12));
}

TEST(ExtrinsicConvention, RotationMapsLaserVectorsIntoImuFrame) {
  const Eigen::Matrix3d R_B_L = rpy(0.2, 0.3, 0.4);
  const Eigen::Vector3d v_laser(1.0, 0.0, 0.0);

  const Eigen::Vector3d v_imu = R_B_L * v_laser;
  EXPECT_NEAR(v_imu.norm(), 1.0, 1e-12);
  EXPECT_TRUE((R_B_L.inverse() * v_imu).isApprox(v_laser, 1e-12));
}

TEST(CameraProjection, PinholeProjectNormalizesByDepth) {
  // Minimal pinhole check; camera-model tests expand once the VINS core lands (Phase 2).
  const Eigen::Matrix3d K =
      (Eigen::Matrix3d() << 400, 0, 320, 0, 400, 240, 0, 0, 1).finished();
  const Eigen::Vector3d p_c(0.5, -0.25, 2.0);

  const Eigen::Vector2d uv((K * p_c).hnormalized());
  EXPECT_NEAR(uv.x(), 320.0 + 400.0 * 0.5 / 2.0, 1e-12);
  EXPECT_NEAR(uv.y(), 240.0 + 400.0 * (-0.25) / 2.0, 1e-12);
}
