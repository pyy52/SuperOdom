//
// Minimal std_msgs::Header stub for the vendored VINS estimator core.
// Upstream code only consumes header.stamp.toSec() (measurement time);
// the estimator node adapter passes measurement timestamps through this stub.
//
#pragma once

namespace std_msgs {

struct Time
{
    double sec = 0.0;
    double toSec() const { return sec; }
};

struct Header
{
    Time stamp;
    std::string frame_id;
};

}  // namespace std_msgs
