#pragma once
#include <fstream>
#include <string>
#include <mutex>
#include <memory>
#include <sstream>
#include <gtsam/geometry/Pose3.h>

namespace super_odometry_vio {
namespace fusion_2021 {

class ParityTracer {
public:
    ParityTracer(const std::string& filename) {
        if (!filename.empty()) {
            out_.open(filename, std::ios::out | std::ios::trunc);
        }
    }
    
    ~ParityTracer() {
        if (out_.is_open()) out_.close();
    }

    bool active() const { return out_.is_open(); }

    void write(const std::string& type, int64_t timestamp_ns, int k, const std::string& source, int epoch, const std::string& payload_json) {
        if (!active()) return;
        std::lock_guard<std::mutex> lock(m_);
        out_ << "{\"type\":\"" << type 
             << "\",\"timestamp_ns\":" << timestamp_ns;
        
        if (k >= 0) {
            out_ << ",\"k\":" << k;
        } else {
            out_ << ",\"k\":null";
        }
        
        out_ << ",\"source\":\"" << source 
             << "\",\"epoch\":" << epoch 
             << ",\"payload\":" << payload_json << "}\n";
        out_.flush();
    }

    static std::string poseToJson(const gtsam::Pose3& p) {
        std::stringstream ss;
        ss << "[" << p.x() << "," << p.y() << "," << p.z() << ","
           << p.rotation().toQuaternion().x() << ","
           << p.rotation().toQuaternion().y() << ","
           << p.rotation().toQuaternion().z() << ","
           << p.rotation().toQuaternion().w() << "]";
        return ss.str();
    }

    static std::string vec3ToJson(const gtsam::Vector3& v) {
        std::stringstream ss;
        ss << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
        return ss.str();
    }

    static std::string vec6ToJson(const gtsam::Vector6& v) {
        std::stringstream ss;
        ss << "[" << v(0) << "," << v(1) << "," << v(2) << ","
           << v(3) << "," << v(4) << "," << v(5) << "]";
        return ss.str();
    }
    
    // Helper to format string arrays safely
    static std::string stringArrayToJson(const std::vector<std::string>& arr) {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < arr.size(); ++i) {
            ss << "\"" << arr[i] << "\"";
            if (i + 1 < arr.size()) ss << ",";
        }
        ss << "]";
        return ss.str();
    }

private:
    std::ofstream out_;
    std::mutex m_;
};

}
}
