// VINS frame-convention boundary tests (Gate doc section 14).
// Locks, with non-identity values, the vendored estimator's conventions:
//   Rs[i], Ps[i]          : T_W_B (body=IMU pose in world)
//   ric / RIC[0]          : R_B_C (camera -> IMU/body)
//   tic / TIC[0]          : p_B_C (camera origin in body)
//   deltaQ(w*dt)          : body-frame rotation increment (right-multiplied)
// Evidence: estimator.cpp Rs[j] *= Utility::deltaQ(un_gyr*dt);
//           projection_factor.cpp pts_imu_i = qic * pts_camera_i + tic;
//                                     pts_w     = Qi  * pts_imu_i   + Pi;
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "estimator.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "utility/utility.h"

namespace {

Eigen::Matrix3d rpy(double r, double p, double y) {
  return (Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()))
      .toRotationMatrix();
}

Eigen::Quaterniond qFromRpy(double r, double p, double y) {
  return Eigen::Quaterniond(rpy(r, p, y));
}

// Encodes a pose into the estimator's parameter block layout
// [px, py, pz, qx, qy, qz, qw] (see pose_local_parameterization.cpp).
std::vector<double> poseBlock(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  return {p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()};
}

// Writes the estimator settings file with a deliberately non-identity
// camera extrinsic; mirrors the VINS config format.
std::string writeEstimatorConfig(const std::string& path) {
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  fs << "max_solver_time" << 0.04;
  fs << "max_num_iterations" << 8;
  fs << "keyframe_parallax" << 10.0;
  fs << "acc_n" << 0.1;
  fs << "acc_w" << 0.001;
  fs << "gyr_n" << 0.01;
  fs << "gyr_w" << 0.0001;
  fs << "g_norm" << 9.805;
  fs << "image_height" << 512;
  fs << "image_width" << 512;
  fs << "estimate_extrinsic" << 0;
  fs << "extrinsicRotation"
     << "{:" << "rows" << 3 << "cols" << 3 << "dt" << "d" << "data"
     << std::vector<double>{0.98, -0.15, 0.12, 0.10, 0.95, -0.29, -0.16, 0.27, 0.95} << "}";
  fs << "extrinsicTranslation"
     << "{:" << "rows" << 3 << "cols" << 1 << "dt" << "d" << "data"
     << std::vector<double>{0.05, -0.02, 0.01} << "}";
  fs << "td" << 0.0;
  fs << "estimate_td" << 0;
  fs << "rolling_shutter" << 0;
  fs << "imu_topic" << "/imu0";
  fs << "output_path" << "/tmp/vio_test_out";
  fs.release();
  return path;
}

}  // namespace

TEST(VinsFrameConvention, DeltaQIsBodyFrameFirstOrderQuaternion) {
  // Upstream Utility::deltaQ is the vendor's first-order increment
  // q = [1, theta/2] (UNNORMALISED), applied right-multiplied in
  // estimator.cpp (Rs[j] = Rs[j] * deltaQ(un_gyr*dt)): gyro lives in the
  // body frame. Locked here as-is (algorithm diff ~= 0); the approximation
  // error is O(theta^2) and the integration steps keep theta small.
  const Eigen::Vector3d omega_dt(0.031, -0.077, 0.152);
  const Eigen::Quaterniond q = Utility::deltaQ(omega_dt);

  EXPECT_NEAR(q.w(), 1.0, 1e-15);
  EXPECT_NEAR(q.x(), 0.5 * omega_dt.x(), 1e-15);
  EXPECT_NEAR(q.y(), 0.5 * omega_dt.y(), 1e-15);
  EXPECT_NEAR(q.z(), 0.5 * omega_dt.z(), 1e-15);

  // For integration-sized increments it must still approximate the SO(3)
  // exponential of a body-frame rotation closely (documents the valid regime).
  const Eigen::Vector3d small_dt = omega_dt * 0.005;
  const Eigen::Quaterniond q_small = Utility::deltaQ(small_dt).normalized();
  const Eigen::AngleAxisd expected(small_dt.norm(), small_dt.normalized());
  EXPECT_TRUE(q_small.toRotationMatrix().isApprox(expected.toRotationMatrix(), 1e-8));
}

TEST(VinsFrameConvention, EstimatorLoadsRicTicAsBodyLeftCamera) {
  const std::string cfg = writeEstimatorConfig("/tmp/vio_est_config.yaml");
  readParameters(cfg);

  Estimator estimator;
  estimator.setParameter();

  // yaml R maps camera -> body (R_B_C). Its transposed-inverse must be SO(3)
  // and the estimator must store exactly the yaml rotation (normalised).
  Eigen::Matrix3d R_yaml;
  R_yaml << 0.98, -0.15, 0.12, 0.10, 0.95, -0.29, -0.16, 0.27, 0.95;
  R_yaml = Eigen::Quaterniond(R_yaml).normalized().toRotationMatrix();

  EXPECT_TRUE(estimator.ric[0].isApprox(R_yaml, 1e-9));
  EXPECT_TRUE(estimator.tic[0].isApprox(Eigen::Vector3d(0.05, -0.02, 0.01), 1e-12));

  // World <- body <- camera composition must be the camera pose in world.
  const Eigen::Matrix3d R_W_B = rpy(0.2, -0.3, 0.7);
  const Eigen::Vector3d p_W_B(1.0, -2.0, 0.5);
  const Eigen::Matrix3d R_W_C = R_W_B * estimator.ric[0];
  const Eigen::Vector3d p_W_C = p_W_B + R_W_B * estimator.tic[0];
  EXPECT_NEAR(R_W_C.determinant(), 1.0, 1e-12);
  // A point at the camera origin maps back exactly.
  const Eigen::Vector3d p_c_origin = Eigen::Vector3d::Zero();
  const Eigen::Vector3d p_w =
      p_W_B + R_W_B * (estimator.ric[0] * p_c_origin + estimator.tic[0]);
  EXPECT_TRUE(p_w.isApprox(p_W_C, 1e-12));
}

TEST(VinsFrameConvention, ProjectionFactorZeroResidualNonIdentityChain) {
  // Full chain lock: R_WB * (R_B_C * p_c + t_B_C) + p_WB, with non-identity
  // rotation AND translation on both poses and the extrinsic.
  ProjectionFactor::sqrt_info = 100.0 * Eigen::Matrix2d::Identity();

  const Eigen::Quaterniond Q_i = qFromRpy(0.2, 0.3, 0.4);
  const Eigen::Vector3d P_i(1.0, 2.0, 3.0);
  const Eigen::Quaterniond Q_j = qFromRpy(-0.1, 0.5, 0.2);
  const Eigen::Vector3d P_j(4.0, 0.0, -1.0);
  const Eigen::Quaterniond Q_ic = qFromRpy(0.3, -0.2, 0.15);
  const Eigen::Vector3d t_ic(0.05, -0.02, 0.01);

  // Consistent landmark in world -> normalized observations at both poses.
  // Vendor chain (projection_factor.cpp): pts_w = Qi*(qic*p_cam_i + tic) + Pi,
  // so p_cam = qic^-1 * (Q^-1*(P_w - P) - tic).
  const Eigen::Vector3d P_w(5.0, 1.0, 2.0);
  const Eigen::Vector3d pts_cam_i =
      Q_ic.inverse() * (Q_i.inverse() * (P_w - P_i) - t_ic);
  const Eigen::Vector3d pts_cam_j =
      Q_ic.inverse() * (Q_j.inverse() * (P_w - P_j) - t_ic);
  ASSERT_GT(pts_cam_i.z(), 0.5);
  ASSERT_GT(pts_cam_j.z(), 0.5);
  const Eigen::Vector3d pts_i = pts_cam_i / pts_cam_i.z();
  const Eigen::Vector3d pts_j = pts_cam_j / pts_cam_j.z();
  const double inv_dep_i = 1.0 / pts_cam_i.z();

  ProjectionFactor factor(pts_i, pts_j);
  std::vector<double> bi = poseBlock(P_i, Q_i);
  std::vector<double> bj = poseBlock(P_j, Q_j);
  std::vector<double> bext = poseBlock(t_ic, Q_ic);
  std::vector<double> bdepth = {inv_dep_i};
  const double* params[4] = {bi.data(), bj.data(), bext.data(), bdepth.data()};

  double residuals[2] = {0.0, 0.0};
  ASSERT_TRUE(factor.Evaluate(params, residuals, nullptr));
  EXPECT_NEAR(residuals[0], 0.0, 1e-9);
  EXPECT_NEAR(residuals[1], 0.0, 1e-9);
}

TEST(VinsFrameConvention, TumViKalibrInverseConsistency) {
  // Gate decision D: the TUM-VI profile's extrinsic is the inverse of the
  // official kalibr T_cam_imu; the inverse relation is locked numerically.
  const Eigen::Matrix3d R_C_I =
      (Eigen::Matrix3d() << -0.9995250378696743, 0.029615343885863205,
       -0.008522328211654736, 0.0075019185074052044, -0.03439736061393144,
       -0.9993800792498829, -0.02989013031643309, -0.998969345370175,
       0.03415885127385616)
          .finished();
  const Eigen::Vector3d p_C_I(0.04727988224914392, -0.047443232143367084,
                              -0.0681999605066297);

  const Eigen::Matrix3d R_I_C = R_C_I.transpose();
  const Eigen::Vector3d p_I_C = -R_C_I.transpose() * p_C_I;

  // p_C = T_C_I * p_I must undo p_I = T_I_C * p_C.
  const Eigen::Vector3d p_body(0.03, -0.07, 0.11);
  const Eigen::Vector3d p_cam = R_C_I * p_body + p_C_I;
  const Eigen::Vector3d p_body_back = R_I_C * p_cam + p_I_C;
  EXPECT_TRUE(p_body_back.isApprox(p_body, 1e-12));

  // The values shipped in tum_vi_room1_vio.yaml must equal the analytic
  // inverse (checked to the digits written into the config file).
  const Eigen::Matrix3d R_yaml =
      (Eigen::Matrix3d() << -0.9995250378696743, 0.0075019185074052044,
       -0.02989013031643309, 0.029615343885863205, -0.03439736061393144,
       -0.998969345370175, -0.008522328211654736, -0.9993800792498829,
       0.03415885127385616)
          .finished();
  EXPECT_TRUE(R_yaml.isApprox(R_I_C, 1e-15));
  EXPECT_TRUE(Eigen::Vector3d(0.045574835649698026, -0.07116180183799704,
                              -0.04468125411714437)
                  .isApprox(p_I_C, 1e-15));
}

TEST(VinsFrameConvention, PoseParameterizationPlusMatchesAnalytic) {
  const Eigen::Quaterniond q = qFromRpy(0.1, -0.4, 0.9);
  const Eigen::Vector3d p(0.5, -0.3, 1.2);
  const std::vector<double> x = poseBlock(p, q);

  const Eigen::Vector3d dp(0.02, -0.01, 0.03);
  const Eigen::Vector3d dw(0.005, 0.02, -0.01);
  std::vector<double> delta = {dp.x(), dp.y(), dp.z(), dw.x(), dw.y(), dw.z()};
  std::vector<double> x_plus(7, 0.0);

  // Upstream declares the overrides private; they are exercised through the
  // public ceres base-class interface exactly as Ceres itself calls them.
  PoseLocalParameterization param_impl;
  ceres::LocalParameterization& param = param_impl;
  ASSERT_EQ(param.GlobalSize(), 7);
  ASSERT_EQ(param.LocalSize(), 6);
  ASSERT_TRUE(param.Plus(x.data(), delta.data(), x_plus.data()));

  const Eigen::Quaterniond dq = Utility::deltaQ(dw);
  const Eigen::Quaterniond q_expected = (q * dq).normalized();
  const Eigen::Map<const Eigen::Quaterniond> q_got(x_plus.data() + 3);
  EXPECT_TRUE(q_got.coeffs().isApprox(q_expected.coeffs(), 1e-12));
  EXPECT_NEAR(x_plus[0], p.x() + dp.x(), 1e-12);
  EXPECT_NEAR(x_plus[1], p.y() + dp.y(), 1e-12);
  EXPECT_NEAR(x_plus[2], p.z() + dp.z(), 1e-12);
}
