#include <gtest/gtest.h>
#include <fstream>
#include <string>
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

ShadowConfig baseConfig()
{
    ShadowConfig c;
    c.anchor_rate_hz = 10.0;
    c.max_constraint_lateness_ns = 500000000ll;
    c.max_imu_dt_ns = 50000000ll;
    c.max_interpolation_gap_ns = 150000000ll;
    c.source_reorder_horizon_ns = 100000000ll;
    c.innovation_trans_m = 0.5;
    c.innovation_rot_rad = 0.2;
    // Emitting trace for parity
    c.parity_trace_file = "test_shadow_trace.jsonl";
    return c;
}

TEST(ShadowParity, GeneratesExpectedJsonLines) {
    // Remove old trace
    std::remove("test_shadow_trace.jsonl");
    
    ShadowTimeline tl(imuParams(), baseConfig());
    tl.setT_B_L(Sophus::SE3d());

    int64_t t = 1000000000ll;
    
    // Feed 3 IMU samples (dt=50ms) -> crosses an anchor boundary at 1.0, 1.1...
    for (int i = 0; i < 5; ++i) {
        tl.feedImu(t, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
        t += 50000000ll;
    }
    
    // Feed a LIO pose at t = 1.0s (epoch 1)
    tl.feedLioPose(1000000000ll, Sophus::SE3d(), 1);
    
    // Feed a LIO pose at t = 1.1s (epoch 1) -> triggers interpolation bracket search -> accepted
    Sophus::SE3d pose_b;
    pose_b.translation() = Eigen::Vector3d(0.1, 0, 0);
    tl.feedLioPose(1100000000ll, pose_b, 1);
    
    // Push watermark slightly to process 1.1
    tl.feedLioPose(1250000000ll, Sophus::SE3d(), 1);

    // Feed a LIO pose too late
    tl.feedLioPose(900000000ll, Sophus::SE3d(), 1);

    // Feed IMU up to 1.4s
    for (int i = 0; i < 8; ++i) {
        tl.feedImu(t, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
        t += 50000000ll;
    }

    // Feed LIO pose at 1.2s with HUGE error to trigger REJECT_INNOVATION_TRANS
    // Since watermark is 1250 - 100 = 1150, 1.2s is > watermark
    Sophus::SE3d pose_err;
    pose_err.translation() = Eigen::Vector3d(10.0, 0, 0);
    tl.feedLioPose(1200000000ll, pose_err, 1);

    // Push watermark to 1.3s to force tryInsertLioFactors up to 1.2s (1.2 + 0.15 = 1.35s)
    tl.feedLioPose(1500000000ll, Sophus::SE3d(), 1);


    // Read lines
    std::ifstream in("test_shadow_trace.jsonl");
    ASSERT_TRUE(in.is_open());
    
    std::string line;
    int line_count = 0;
    bool found_anchor_open = false;
    bool found_gate_eval = false;
    bool found_factor_insert = false;
    bool found_state_snapshot = false;
    
    while (std::getline(in, line)) {
        line_count++;
        if (line.find("\"type\":\"TIMELINE_ANCHOR_OPEN\"") != std::string::npos) found_anchor_open = true;
        if (line.find("\"type\":\"GATE_EVALUATION\"") != std::string::npos) found_gate_eval = true;
        if (line.find("\"type\":\"FACTOR_INSERT\"") != std::string::npos) found_factor_insert = true;
        if (line.find("\"type\":\"STATE_SNAPSHOT\"") != std::string::npos) found_state_snapshot = true;
    }
    
    EXPECT_GT(line_count, 5);
    EXPECT_TRUE(found_anchor_open);
    EXPECT_TRUE(found_gate_eval);
    EXPECT_TRUE(found_factor_insert);
    EXPECT_TRUE(found_state_snapshot);
}

}
