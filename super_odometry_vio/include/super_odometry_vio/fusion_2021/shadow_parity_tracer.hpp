#pragma once

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>

namespace super_odometry_vio {
namespace fusion_2021 {

class ParityTracer {
public:
    explicit ParityTracer(const std::string& filename, const std::string& producer = "shadow")
        : filename_(filename), producer_(producer), has_error_(false)
    {
        if (!filename_.empty()) {
            out_.open(filename_, std::ios::out | std::ios::trunc);
            if (!out_.is_open()) {
                has_error_ = true;
                error_msg_ = "Failed to open parity trace file: " + filename_;
            }
        }
    }

    ~ParityTracer() {
        close();
    }

    void close() {
        std::lock_guard<std::mutex> lock(m_);
        if (out_.is_open()) {
            out_.flush();
            out_.close();
        }
    }

    bool active() const {
        return out_.is_open() && !has_error_;
    }

    bool hasError() const {
        return has_error_;
    }

    std::string errorMessage() const {
        std::lock_guard<std::mutex> lock(m_);
        return error_msg_;
    }

    const std::string& filename() const {
        return filename_;
    }

    const std::string& producer() const {
        return producer_;
    }

    // --- Low-level JSON formatting helpers ---

    static std::string escapeJson(const std::string& s) {
        std::ostringstream o;
        for (char c : s) {
            switch (c) {
                case '"':  o << "\\\""; break;
                case '\\': o << "\\\\"; break;
                case '\b': o << "\\b";  break;
                case '\f': o << "\\f";  break;
                case '\n': o << "\\n";  break;
                case '\r': o << "\\r";  break;
                case '\t': o << "\\t";  break;
                default:
                    if (static_cast<unsigned char>(c) <= 0x1f) {
                        o << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                          << static_cast<int>(static_cast<unsigned char>(c));
                    } else {
                        o << c;
                    }
            }
        }
        return o.str();
    }

    static std::string formatDouble(double val) {
        if (!std::isfinite(val)) {
            throw std::runtime_error("ParityTracer encountered non-finite float value: " + std::to_string(val));
        }
        std::ostringstream ss;
        ss.imbue(std::locale::classic());
        ss << std::setprecision(std::numeric_limits<double>::max_digits10) << val;
        std::string s = ss.str();
        // Ensure floating point numbers contain a decimal point or exponent
        if (s.find('.') == std::string::npos && s.find('e') == std::string::npos && s.find('E') == std::string::npos) {
            s += ".0";
        }
        return s;
    }

    static std::string poseToJson(const gtsam::Pose3& p) {
        // [tx, ty, tz, qx, qy, qz, qw]
        const auto& q = p.rotation().toQuaternion();
        std::ostringstream ss;
        ss.imbue(std::locale::classic());
        ss << "["
           << formatDouble(p.x()) << ","
           << formatDouble(p.y()) << ","
           << formatDouble(p.z()) << ","
           << formatDouble(q.x()) << ","
           << formatDouble(q.y()) << ","
           << formatDouble(q.z()) << ","
           << formatDouble(q.w()) << "]";
        return ss.str();
    }

    static std::string vec3ToJson(const gtsam::Vector3& v) {
        std::ostringstream ss;
        ss.imbue(std::locale::classic());
        ss << "["
           << formatDouble(v.x()) << ","
           << formatDouble(v.y()) << ","
           << formatDouble(v.z()) << "]";
        return ss.str();
    }

    static std::string tangent6ToJson(const gtsam::Vector6& v) {
        // Tangent 6-vector: rotation(3) first, translation(3) second
        std::ostringstream ss;
        ss.imbue(std::locale::classic());
        ss << "["
           << formatDouble(v(0)) << ","
           << formatDouble(v(1)) << ","
           << formatDouble(v(2)) << ","
           << formatDouble(v(3)) << ","
           << formatDouble(v(4)) << ","
           << formatDouble(v(5)) << "]";
        return ss.str();
    }

    static std::string stringArrayToJson(const std::vector<std::string>& arr) {
        std::ostringstream ss;
        ss << "[";
        for (size_t i = 0; i < arr.size(); ++i) {
            ss << "\"" << escapeJson(arr[i]) << "\"";
            if (i + 1 < arr.size()) ss << ",";
        }
        ss << "]";
        return ss.str();
    }

    // Low-level envelope writer
    void writeRaw(const std::string& type, int64_t timestamp_ns, int k,
                  const std::string& source, int epoch, const std::string& payload_json)
    {
        if (!active()) return;
        std::lock_guard<std::mutex> lock(m_);
        out_ << "{\"schema_version\":1,\"producer\":\"" << escapeJson(producer_)
             << "\",\"type\":\"" << escapeJson(type)
             << "\",\"timestamp_ns\":" << timestamp_ns;

        if (k >= 0) {
            out_ << ",\"k\":" << k;
        } else {
            out_ << ",\"k\":null";
        }

        out_ << ",\"source\":\"" << escapeJson(source)
             << "\",\"epoch\":" << epoch
             << ",\"payload\":" << payload_json << "}\n";

        if (!out_.good()) {
            has_error_ = true;
            error_msg_ = "Stream write failure in ParityTracer";
        } else {
            out_.flush();
        }
    }

    // --- Typed Event Methods for 6-Layer Telemetry ---

    // Layer 1: Timeline
    void traceTimelineAnchor(const std::string& type, int64_t timestamp_ns, int k,
                             const std::string& source, int epoch, int64_t t_k,
                             const std::string& anchor_status,
                             const std::string& fusion_segment,
                             const std::string& interval_status)
    {
        std::ostringstream p;
        p << "{\"t_k\":" << t_k
          << ",\"anchor_status\":\"" << escapeJson(anchor_status) << "\""
          << ",\"fusion_segment\":\"" << escapeJson(fusion_segment) << "\""
          << ",\"interval_status\":\"" << escapeJson(interval_status) << "\"}";
        writeRaw(type, timestamp_ns, k, source, epoch, p.str());
    }

    // Layer 2: Input Consumption
    void traceInputConsume(int64_t timestamp_ns, int k, const std::string& source, int epoch,
                           const std::string& sensor_type, int64_t sample_time_ns,
                           const std::string& action, const std::string& drop_reason,
                           bool epoch_changed, int64_t lateness_ns = 0, bool coverage_gap = false)
    {
        std::ostringstream p;
        p << "{\"sensor_type\":\"" << escapeJson(sensor_type) << "\""
          << ",\"sample_time_ns\":" << sample_time_ns
          << ",\"action\":\"" << escapeJson(action) << "\"";
        if (drop_reason.empty() || drop_reason == "none") {
            p << ",\"drop_reason\":null";
        } else {
            p << ",\"drop_reason\":\"" << escapeJson(drop_reason) << "\"";
        }
        p << ",\"epoch_changed\":" << (epoch_changed ? "true" : "false")
          << ",\"lateness_ns\":" << lateness_ns
          << ",\"coverage_gap\":" << (coverage_gap ? "true" : "false") << "}";
        writeRaw("INPUT_CONSUME", timestamp_ns, k, source, epoch, p.str());
    }

    // Layer 3: Factor Insert
    void traceFactorInsert(int64_t timestamp_ns, int k, const std::string& source, int epoch,
                           const std::string& factor_type, const std::vector<std::string>& keys,
                           const gtsam::Pose3& measurement, const gtsam::Vector6& noise_sigmas,
                           bool committed, bool finalized, int64_t insertion_time_ns)
    {
        std::ostringstream p;
        p << "{\"factor_type\":\"" << escapeJson(factor_type) << "\""
          << ",\"keys\":" << stringArrayToJson(keys)
          << ",\"measurement_se3\":" << poseToJson(measurement)
          << ",\"noise_sigmas\":" << tangent6ToJson(noise_sigmas)
          << ",\"committed\":" << (committed ? "true" : "false")
          << ",\"finalized\":" << (finalized ? "true" : "false")
          << ",\"insertion_time_ns\":" << insertion_time_ns << "}";
        writeRaw("FACTOR_INSERT", timestamp_ns, k, source, epoch, p.str());
    }

    // Layer 4: Gate Evaluation
    void traceGateEvaluation(int64_t timestamp_ns, int k, const std::string& source, int epoch,
                             const gtsam::Pose3& dT_imu_ref, const gtsam::Pose3& dT_source,
                             const gtsam::Vector6& innovation6, double trans_norm, double rot_norm,
                             double trans_thresh, double rot_thresh,
                             const std::string& decision, const std::string& reason,
                             int64_t lateness_ns, int64_t watermark_ns,
                             int64_t left_bracket_ns, int64_t right_bracket_ns,
                             bool has_source = true)
    {
        std::ostringstream p;
        p.imbue(std::locale::classic());
        p << "{\"dT_imu_ref\":" << poseToJson(dT_imu_ref);
        if (has_source) {
            p << ",\"dT_source\":" << poseToJson(dT_source)
              << ",\"innovation6\":" << tangent6ToJson(innovation6)
              << ",\"trans_norm\":" << formatDouble(trans_norm)
              << ",\"rot_norm\":" << formatDouble(rot_norm);
        } else {
            p << ",\"dT_source\":null"
              << ",\"innovation6\":null"
              << ",\"trans_norm\":null"
              << ",\"rot_norm\":null";
        }
        p << ",\"thresholds\":[" << formatDouble(trans_thresh) << "," << formatDouble(rot_thresh) << "]"
          << ",\"decision\":\"" << escapeJson(decision) << "\""
          << ",\"reason\":\"" << escapeJson(reason) << "\""
          << ",\"lateness_ns\":" << lateness_ns
          << ",\"watermark_ns\":" << watermark_ns
          << ",\"left_bracket_ns\":" << left_bracket_ns
          << ",\"right_bracket_ns\":" << right_bracket_ns
          << ",\"interpolation_brackets\":[" << left_bracket_ns << "," << right_bracket_ns << "]}";
        writeRaw("GATE_EVALUATION", timestamp_ns, k, source, epoch, p.str());
    }

    // Layer 5: State Snapshot
    void traceStateSnapshot(int64_t timestamp_ns, int k, const std::string& source, int epoch,
                            const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                            const gtsam::Vector3& bias_acc, const gtsam::Vector3& bias_gyro,
                            const std::string& anchor_status, uint64_t optimizer_update_id)
    {
        std::ostringstream p;
        p << "{\"pose\":" << poseToJson(pose)
          << ",\"velocity\":" << vec3ToJson(velocity)
          << ",\"bias_acc\":" << vec3ToJson(bias_acc)
          << ",\"bias_gyro\":" << vec3ToJson(bias_gyro)
          << ",\"anchor_status\":\"" << escapeJson(anchor_status) << "\""
          << ",\"optimizer_update_id\":" << optimizer_update_id << "}";
        writeRaw("STATE_SNAPSHOT", timestamp_ns, k, source, epoch, p.str());
    }

private:
    std::string filename_;
    std::string producer_;
    std::ofstream out_;
    mutable std::mutex m_;
    bool has_error_;
    std::string error_msg_;
};

}  // namespace fusion_2021
}  // namespace super_odometry_vio
