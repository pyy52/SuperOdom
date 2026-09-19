// Camera blackout injector (Phase 2c gate decision E): passes images through
// with UNCHANGED timestamps and rate, replacing frames whose header time falls
// inside configured blackout intervals with black frames. This tests visual
// QUALITY degradation (distinct from Phase 6 message dropout/watchdog tests).
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cstring>
#include <vector>

class CameraBlackoutNode : public rclcpp::Node
{
  public:
    CameraBlackoutNode() : Node("camera_blackout_node")
    {
        in_topic_ = this->declare_parameter<std::string>("in_topic", "/cam0/image_raw");
        out_topic_ = this->declare_parameter<std::string>("out_topic", "/cam0/image_blackout");
        // Comma-separated intervals in seconds relative to the first seen
        // image stamp, e.g. "40:10,90:5" => black [t0+40, t0+50) and [t0+90, t0+95).
        intervals_ = this->declare_parameter<std::string>("intervals", "40:10");

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            in_topic_, 5,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                if (first_stamp_sec_ < 0)
                {
                    first_stamp_sec_ = msg->header.stamp.sec +
                                       1e-9 * msg->header.stamp.nanosec;
                    RCLCPP_INFO(this->get_logger(), "t0 = %.3f, intervals: %s",
                                first_stamp_sec_, intervals_.c_str());
                }
                sensor_msgs::msg::Image out = *msg;  // stamps/rate unchanged
                if (inBlackout(msg->header.stamp.sec +
                               1e-9 * msg->header.stamp.nanosec))
                {
                    out.data.assign(out.data.size(), 0);
                    out.step = out.step;  // layout untouched
                }
                pub_->publish(out);
            });
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_topic_, 5);
        RCLCPP_INFO(this->get_logger(), "blackout injector: %s -> %s",
                    in_topic_.c_str(), out_topic_.c_str());
    }

  private:
    bool inBlackout(double t)
    {
        if (intervals_.empty()) return false;
        const double rel = t - first_stamp_sec_;
        size_t pos = 0;
        while (pos < intervals_.size())
        {
            const size_t colon = intervals_.find(':', pos);
            const size_t comma = intervals_.find(',', pos);
            const size_t end = comma == std::string::npos ? intervals_.size() : comma;
            if (colon == std::string::npos || colon > end)
            {
                pos = end + 1;
                continue;
            }
            const double start = std::atof(intervals_.substr(pos, colon - pos).c_str());
            const double dur = std::atof(intervals_.substr(colon + 1, end - colon - 1).c_str());
            if (rel >= start && rel < start + dur) return true;
            pos = end + 1;
        }
        return false;
    }

    std::string in_topic_, out_topic_, intervals_;
    double first_stamp_sec_{-1.0};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraBlackoutNode>());
    rclcpp::shutdown();
    return 0;
}
