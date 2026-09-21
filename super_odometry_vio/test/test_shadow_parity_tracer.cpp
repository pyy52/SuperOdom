#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <thread>
#include <vector>

#include "super_odometry_vio/fusion_2021/shadow_parity_tracer.hpp"

using namespace super_odometry_vio::fusion_2021;

namespace {

TEST(ParityTracerTest, StringEscaping) {
    std::string unescaped = "Hello \"world\"\n\t\\ \b\f\r test";
    std::string escaped = ParityTracer::escapeJson(unescaped);
    EXPECT_NE(escaped.find("\\\"world\\\""), std::string::npos);
    EXPECT_NE(escaped.find("\\n"), std::string::npos);
    EXPECT_NE(escaped.find("\\t"), std::string::npos);
    EXPECT_NE(escaped.find("\\\\"), std::string::npos);
}

TEST(ParityTracerTest, FloatFormattingAndPrecision) {
    double val = 1.23456789012345e-7;
    std::string formatted = ParityTracer::formatDouble(val);
    double parsed = std::stod(formatted);
    EXPECT_NEAR(val, parsed, 1e-15);

    // Integers formatted as floats have decimal point
    std::string int_as_double = ParityTracer::formatDouble(42.0);
    EXPECT_NE(int_as_double.find('.'), std::string::npos);
}

TEST(ParityTracerTest, NonFiniteThrows) {
    double nan_val = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ParityTracer::formatDouble(nan_val), std::runtime_error);

    double inf_val = std::numeric_limits<double>::infinity();
    EXPECT_THROW(ParityTracer::formatDouble(inf_val), std::runtime_error);
}

TEST(ParityTracerTest, EmptyPathDoesNotCreateFile) {
    ParityTracer tracer("");
    EXPECT_FALSE(tracer.active());
    EXPECT_FALSE(tracer.hasError());
}

TEST(ParityTracerTest, InvalidPathSetsError) {
    // Non-existent nested directory
    ParityTracer tracer("/non_existent_directory_abc_xyz/cannot_create.jsonl");
    EXPECT_FALSE(tracer.active());
    EXPECT_TRUE(tracer.hasError());
    EXPECT_FALSE(tracer.errorMessage().empty());
}

TEST(ParityTracerTest, ThreadSafetyAndDeterministicOutput) {
    const std::string file1 = "/tmp/test_parity_tracer_thread1.jsonl";
    const std::string file2 = "/tmp/test_parity_tracer_thread2.jsonl";
    std::remove(file1.c_str());
    std::remove(file2.c_str());

    auto run_traces = [](const std::string& filepath) {
        ParityTracer tracer(filepath, "shadow");
        std::vector<std::thread> threads;
        for (int t = 0; t < 4; ++t) {
            threads.emplace_back([&tracer, t]() {
                for (int i = 0; i < 25; ++i) {
                    tracer.traceTimelineAnchor("TIMELINE_ANCHOR_OPEN", 1000000000 + i * 100000000ll, i,
                                               "IMU", 0, 1000000000 + i * 100000000ll,
                                               "GRAPH_INSERTED", "ACTIVE", "OPEN");
                }
            });
        }
        for (auto& th : threads) {
            th.join();
        }
        tracer.close();
    };

    run_traces(file1);

    // Verify file exists and has 100 lines
    std::ifstream in(file1);
    ASSERT_TRUE(in.is_open());
    std::string line;
    int line_count = 0;
    while (std::getline(in, line)) {
        ++line_count;
        EXPECT_NE(line.find("\"schema_version\":1"), std::string::npos);
        EXPECT_NE(line.find("\"producer\":\"shadow\""), std::string::npos);
    }
    EXPECT_EQ(line_count, 100);

    std::remove(file1.c_str());
    std::remove(file2.c_str());
}

}  // namespace
