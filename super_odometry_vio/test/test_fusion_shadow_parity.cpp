#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "super_odometry_vio/fusion_2021/shadow_timeline.hpp"

using namespace super_odometry_vio::fusion_2021;
using namespace gtsam;

namespace {

std::shared_ptr<gtsam::PreintegrationParams> imuParams()
{
    auto p = gtsam::PreintegrationParams::MakeSharedU(9.81);
    p->accelerometerCovariance = gtsam::Matrix33::Identity() * 1e-3;
    p->gyroscopeCovariance = gtsam::Matrix33::Identity() * 1e-4;
    p->integrationCovariance = gtsam::Matrix33::Identity() * 1e-8;
    return p;
}

ShadowConfig baseConfig(const std::string& trace_file = "")
{
    ShadowConfig c;
    c.anchor_rate_hz = 10.0;
    c.max_constraint_lateness_ns = 500000000ll;
    c.max_imu_dt_ns = 50000000ll;
    c.max_interpolation_gap_ns = 150000000ll;
    c.source_reorder_horizon_ns = 100000000ll;
    c.innovation_trans_m = 0.5;
    c.innovation_rot_rad = 0.2;
    c.parity_trace_file = trace_file;
    return c;
}

// Helper to run a deterministic synthetic scenario
void runSyntheticScenario(ShadowTimeline& tl)
{
    // Extrinsic calibration
    Sophus::SE3d T_B_L;
    T_B_L.translation() = Eigen::Vector3d(0.05, 0.0, 0.02);
    tl.setT_B_L(T_B_L);

    int64_t t = 1000000000ll;

    // Feed non-trivial IMU measurements with acceleration and gyro rotation
    for (int i = 0; i <= 10; ++i) {
        tl.feedImu(t, gtsam::Vector3(0.5, 0.1, 9.81), gtsam::Vector3(0.05, 0.02, 0.1));
        t += 50000000ll;
    }

    // Feed LIO pose at t = 1.0s (epoch 1)
    Sophus::SE3d pose_0;
    pose_0.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0.01, 0.0, 0.02));
    tl.feedLioPose(1000000000ll, pose_0, 1);

    // Feed LIO pose at t = 1.1s (epoch 1)
    Sophus::SE3d pose_1;
    pose_1.translation() = Eigen::Vector3d(0.08, 0.01, 0.0);
    pose_1.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0.02, 0.01, 0.03));
    tl.feedLioPose(1100000000ll, pose_1, 1);

    // Feed LIO pose at t = 1.2s (epoch 1)
    Sophus::SE3d pose_2;
    pose_2.translation() = Eigen::Vector3d(0.16, 0.02, 0.0);
    pose_2.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0.03, 0.02, 0.04));
    tl.feedLioPose(1200000000ll, pose_2, 1);

    // Feed LIO pose at t = 1.3s with excessive translation to trigger innovation rejection
    Sophus::SE3d pose_bad;
    pose_bad.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
    pose_bad.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0.04, 0.02, 0.05));
    tl.feedLioPose(1300000000ll, pose_bad, 1);

    // Push watermark to evaluate constraints up to 1.3s (watermark >= 1.3s + 150ms = 1.45s -> stamp >= 1.55s)
    Sophus::SE3d pose_push;
    pose_push.translation() = Eigen::Vector3d(0.24, 0.03, 0.0);
    tl.feedLioPose(1550000000ll, pose_push, 1);

    // Feed a late LIO pose (watermark is 1.55s - 0.1s = 1.45s, so 1.35s is late and dropped by watermark)
    tl.feedLioPose(1350000000ll, pose_0, 1);

    // Feed a stale epoch LIO pose (epoch 0 < current epoch 1)
    tl.feedLioPose(1600000000ll, pose_1, 0);
}

TEST(ShadowParity, F1_StrictSchemaComplianceAndStructure) {
    const std::string trace_path = "/tmp/test_parity_f1.jsonl";
    std::remove(trace_path.c_str());

    ShadowTimeline tl(imuParams(), baseConfig(trace_path));
    runSyntheticScenario(tl);

    std::ifstream in(trace_path);
    ASSERT_TRUE(in.is_open());

    std::string line;
    int line_count = 0;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        ++line_count;
        EXPECT_NE(line.find("\"schema_version\":1"), std::string::npos);
        EXPECT_NE(line.find("\"producer\":\"shadow\""), std::string::npos);
        EXPECT_NE(line.find("\"type\":"), std::string::npos);
        EXPECT_NE(line.find("\"timestamp_ns\":"), std::string::npos);
        EXPECT_NE(line.find("\"source\":"), std::string::npos);
        EXPECT_NE(line.find("\"epoch\":"), std::string::npos);
        EXPECT_NE(line.find("\"payload\":"), std::string::npos);
    }
    EXPECT_GT(line_count, 10);
    std::remove(trace_path.c_str());
}

TEST(ShadowParity, F2_NonTrivialPhysicalValuesNoPlaceholderZeros) {
    const std::string trace_path = "/tmp/test_parity_f2.jsonl";
    std::remove(trace_path.c_str());

    ShadowTimeline tl(imuParams(), baseConfig(trace_path));
    runSyntheticScenario(tl);

    std::ifstream in(trace_path);
    ASSERT_TRUE(in.is_open());

    std::string line;
    bool checked_gate = false;
    bool checked_factor = false;

    while (std::getline(in, line)) {
        if (line.find("\"type\":\"GATE_EVALUATION\"") != std::string::npos &&
            line.find("\"decision\":\"ACCEPTED\"") != std::string::npos) {
            checked_gate = true;
            // Ensure no placeholder zeros for dT_imu_ref or dT_source
            EXPECT_EQ(line.find("\"dT_imu_ref\":0"), std::string::npos);
            EXPECT_EQ(line.find("\"dT_source\":0"), std::string::npos);
            // Has genuine translation norm > 0
            EXPECT_NE(line.find("\"trans_norm\":"), std::string::npos);
            EXPECT_EQ(line.find("\"trans_norm\":0.0,"), std::string::npos);
        }
        if (line.find("\"type\":\"FACTOR_INSERT\"") != std::string::npos) {
            checked_factor = true;
            EXPECT_NE(line.find("\"measurement_se3\":["), std::string::npos);
            EXPECT_NE(line.find("\"noise_sigmas\":["), std::string::npos);
            EXPECT_NE(line.find("\"committed\":true"), std::string::npos);
        }
    }
    EXPECT_TRUE(checked_gate);
    EXPECT_TRUE(checked_factor);
    std::remove(trace_path.c_str());
}

TEST(ShadowParity, F3_LifecycleAndTerminalStates) {
    const std::string trace_path = "/tmp/test_parity_f3.jsonl";
    std::remove(trace_path.c_str());

    ShadowTimeline tl(imuParams(), baseConfig(trace_path));
    runSyntheticScenario(tl);

    std::ifstream in(trace_path);
    ASSERT_TRUE(in.is_open());

    bool found_accepted = false;
    bool found_dropped_watermark = false;
    bool found_dropped_stale = false;
    bool found_innovation_reject = false;
    bool found_state_solved = false;

    std::string line;
    while (std::getline(in, line)) {
        if (line.find("\"action\":\"DROPPED_WATERMARK\"") != std::string::npos) {
            found_dropped_watermark = true;
        }
        if (line.find("\"action\":\"DROPPED_STALE\"") != std::string::npos) {
            found_dropped_stale = true;
        }
        if (line.find("\"reason\":\"INNOVATION_TOO_LARGE_TRANS\"") != std::string::npos) {
            found_innovation_reject = true;
        }
        if (line.find("\"decision\":\"ACCEPTED\"") != std::string::npos) {
            found_accepted = true;
        }
        if (line.find("\"anchor_status\":\"SOLVED\"") != std::string::npos) {
            found_state_solved = true;
        }
    }

    EXPECT_TRUE(found_accepted);
    EXPECT_TRUE(found_dropped_watermark);
    EXPECT_TRUE(found_dropped_stale);
    EXPECT_TRUE(found_innovation_reject);
    EXPECT_TRUE(found_state_solved);

    std::remove(trace_path.c_str());
}

TEST(ShadowParity, F4_TracingNonInterferenceABEquivalence) {
    // Run A: Tracing DISABLED (empty file path)
    ShadowTimeline tl_a(imuParams(), baseConfig(""));
    runSyntheticScenario(tl_a);

    // Run B: Tracing ENABLED
    const std::string trace_path = "/tmp/test_parity_f4.jsonl";
    std::remove(trace_path.c_str());
    ShadowTimeline tl_b(imuParams(), baseConfig(trace_path));
    runSyntheticScenario(tl_b);

    // Compare diagnostics counters
    EXPECT_EQ(tl_a.diag().anchors_created, tl_b.diag().anchors_created);
    EXPECT_EQ(tl_a.diag().intervals_closed, tl_b.diag().intervals_closed);
    EXPECT_EQ(tl_a.diag().lio_accepted, tl_b.diag().lio_accepted);
    EXPECT_EQ(tl_a.diag().lio_late_after_watermark, tl_b.diag().lio_late_after_watermark);
    EXPECT_EQ(tl_a.diag().lio_stale_skipped, tl_b.diag().lio_stale_skipped);
    EXPECT_EQ(tl_a.diag().lio_rejected_innovation_trans, tl_b.diag().lio_rejected_innovation_trans);
    EXPECT_EQ(tl_a.anchorCount(), tl_b.anchorCount());

    // Compare solved anchor states exactly
    for (int k = 0; k < tl_a.anchorCount(); ++k) {
        gtsam::Pose3 pose_a, pose_b;
        gtsam::Vector3 vel_a, vel_b;
        gtsam::imuBias::ConstantBias bias_a, bias_b;

        bool solved_a = tl_a.anchorState(k, pose_a, vel_a, bias_a);
        bool solved_b = tl_b.anchorState(k, pose_b, vel_b, bias_b);
        EXPECT_EQ(solved_a, solved_b);

        if (solved_a && solved_b) {
            EXPECT_NEAR((pose_a.translation() - pose_b.translation()).norm(), 0.0, 1e-12);
            EXPECT_NEAR(pose_a.rotation().toQuaternion().angularDistance(pose_b.rotation().toQuaternion()), 0.0, 1e-12);
            EXPECT_NEAR((vel_a - vel_b).norm(), 0.0, 1e-12);
            EXPECT_NEAR((bias_a.vector() - bias_b.vector()).norm(), 0.0, 1e-12);
        }
    }

    std::remove(trace_path.c_str());
}

TEST(ShadowParity, F5_DeterministicReplay) {
    const std::string trace1 = "/tmp/test_parity_run1.jsonl";
    const std::string trace2 = "/tmp/test_parity_run2.jsonl";
    std::remove(trace1.c_str());
    std::remove(trace2.c_str());

    {
        ShadowTimeline tl1(imuParams(), baseConfig(trace1));
        runSyntheticScenario(tl1);
    }
    {
        ShadowTimeline tl2(imuParams(), baseConfig(trace2));
        runSyntheticScenario(tl2);
    }

    std::ifstream in1(trace1);
    std::ifstream in2(trace2);
    ASSERT_TRUE(in1.is_open());
    ASSERT_TRUE(in2.is_open());

    std::string line1, line2;
    int line_idx = 0;
    while (std::getline(in1, line1)) {
        ASSERT_TRUE(std::getline(in2, line2));
        EXPECT_EQ(line1, line2) << "Deterministic mismatch at line " << line_idx;
        ++line_idx;
    }
    EXPECT_FALSE(std::getline(in2, line2));
    EXPECT_GT(line_idx, 10);

    std::remove(trace1.c_str());
    std::remove(trace2.c_str());
}

}  // namespace
