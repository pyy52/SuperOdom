// ROS2 adapter for the vendored VINS sliding-window estimator.
// Image/feature topics in, VioState/health/trajectory out. All estimator
// access is funnelled through one worker thread: the core is not thread-safe
// and measurement-time ordering (IMU before features) must be preserved
// (Phase 2c gate doc sections 12, 13).
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <fstream>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include "super_odometry_vio/vio_estimator.hpp"

using namespace super_odometry_vio;

namespace {

int64_t stampToNs(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<int64_t>(stamp.sec) * 1000000000ll + stamp.nanosec;
}

}  // namespace

class VioEstimatorNode : public rclcpp::Node
{
  public:
    VioEstimatorNode() : Node("vio_estimator_node")
    {
        const std::string config_file =
            this->declare_parameter<std::string>("config_file", "");
        if (config_file.empty())
            throw std::runtime_error("parameter 'config_file' is required");

        VioWrapperConfig wcfg;
        wcfg.trajectory_csv =
            this->declare_parameter<std::string>("trajectory_csv", "");
        wrapper_.setConfig(wcfg);

        if (!wrapper_.loadParameters(config_file))
            throw std::runtime_error("failed to load " + config_file);

        // Fresh trajectory file per run.
        if (!wcfg.trajectory_csv.empty())
            std::ofstream(wcfg.trajectory_csv, std::ios::trunc);

        const std::string imu_topic =
            this->declare_parameter<std::string>("imu_topic", wrapper_.imuTopic());
        const std::string feature_topic = this->declare_parameter<std::string>(
            "feature_topic", "/vio_feature_tracker_node/feature");

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 200,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                ImuSample s;
                s.stamp_ns = stampToNs(msg->header.stamp);
                s.accel << msg->linear_acceleration.x,
                    msg->linear_acceleration.y, msg->linear_acceleration.z;
                s.gyro << msg->angular_velocity.x, msg->angular_velocity.y,
                    msg->angular_velocity.z;
                wrapper_.feedImu(s);
                {
                    std::lock_guard<std::mutex> lk(work_mutex_);
                    ++work_pending_;
                }
                work_cv_.notify_one();
            });

        sub_feature_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            feature_topic, 5,
            [this](const sensor_msgs::msg::PointCloud::SharedPtr msg)
            {
                TrackedFeatureFrame frame;
                frame.stamp_ns = stampToNs(msg->header.stamp);
                const auto& ids = msg->channels[0].values;
                const auto& us = msg->channels[1].values;
                const auto& vs = msg->channels[2].values;
                const auto& vxs = msg->channels[3].values;
                const auto& vys = msg->channels[4].values;
                frame.observations.reserve(msg->points.size());
                for (size_t j = 0; j < msg->points.size(); ++j)
                {
                    TrackedFeatureObservation obs;
                    obs.feature_id = static_cast<int>(ids[j]);
                    obs.bearing << msg->points[j].x, msg->points[j].y,
                        msg->points[j].z;
                    obs.pixel << us[j], vs[j];
                    obs.velocity << vxs[j], vys[j];
                    frame.observations.push_back(obs);
                }
                {
                    std::lock_guard<std::mutex> lk(work_mutex_);
                    feature_queue_.push_back(std::move(frame));
                    while (feature_queue_.size() > 20)
                        feature_queue_.pop_front();  // drop stale frames
                    ++work_pending_;
                }
                work_cv_.notify_one();
            });

        pub_odometry_ =
            this->create_publisher<nav_msgs::msg::Odometry>("~/vio_odometry", 5);
        pub_health_ =
            this->create_publisher<std_msgs::msg::String>("~/vio_health", 5);

        worker_ = std::thread([this]() { workerLoop(); });

        RCLCPP_INFO(this->get_logger(),
                    "vio estimator ready (imu: %s, features: %s)",
                    imu_topic.c_str(), feature_topic.c_str());
    }

    ~VioEstimatorNode() override
    {
        running_ = false;
        work_cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

  private:
    void workerLoop()
    {
        while (running_)
        {
            {
                std::unique_lock<std::mutex> lk(work_mutex_);
                work_cv_.wait(lk, [this]() { return work_pending_ > 0 || !running_; });
            }
            if (!running_) break;

            // NOTE: no blind IMU propagation here. IMU is consumed strictly
            // in measurement-time sync with feature frames inside
            // processFeatures(); draining ahead would mark queued frames
            // stale. High-rate repropagation is a Phase 4 timeline item.
            TrackedFeatureFrame frame;
            bool has_frame = false;
            {
                std::lock_guard<std::mutex> lk(work_mutex_);
                if (!feature_queue_.empty())
                {
                    frame = std::move(feature_queue_.front());
                    feature_queue_.pop_front();
                    has_frame = true;
                }
            }
            if (has_frame)
            {
                wrapper_.processFeatures(frame);
                publishOutputs();
            }
            {
                std::lock_guard<std::mutex> lk(work_mutex_);
                work_pending_ = 0;
            }
        }
    }

    void publishOutputs()
    {
        const VioState st = wrapper_.latestState();
        const VioQuality q = wrapper_.latestQuality();

        nav_msgs::msg::Odometry odom;
        const rclcpp::Time t(st.stamp_ns);
        odom.header.stamp = t;
        odom.header.frame_id = "world";
        odom.child_frame_id = "body";
        odom.pose.pose.position.x = st.T_W_B.translation().x();
        odom.pose.pose.position.y = st.T_W_B.translation().y();
        odom.pose.pose.position.z = st.T_W_B.translation().z();
        const Eigen::Quaterniond q_wb(st.T_W_B.rotationMatrix());
        odom.pose.pose.orientation.x = q_wb.x();
        odom.pose.pose.orientation.y = q_wb.y();
        odom.pose.pose.orientation.z = q_wb.z();
        odom.pose.pose.orientation.w = q_wb.w();
        odom.twist.twist.linear.x = st.velocity_W.x();
        odom.twist.twist.linear.y = st.velocity_W.y();
        odom.twist.twist.linear.z = st.velocity_W.z();
        pub_odometry_->publish(odom);

        char buf[512];
        snprintf(buf, sizeof(buf),
                 "{\"t\":%.3f,\"health\":\"%s\",\"features\":%d,\"inliers\":%d,"
                 "\"parallax_px\":%.2f,\"solve_p50_ms\":%.2f,\"solve_p95_ms\":%.2f,\"info_min_eig\":%.4g,"
                 "\"info_cond\":%.4g,\"observable_dof\":%d,\"optimizer\":%s}",
                 static_cast<double>(st.stamp_ns) * 1e-9, toCString(q.health),
                 q.tracked_features, q.inlier_features,
                 q.median_parallax_px, q.solve_time_ms_p50,
                 q.solve_time_ms_p95, q.information_min_eigenvalue,
                 q.information_condition_number, q.observable_dof,
                 q.optimizer_success ? "true" : "false");
        std_msgs::msg::String health;
        health.data = buf;
        pub_health_->publish(health);
    }

    VioEstimator wrapper_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_feature_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_health_;

    std::thread worker_;
    std::mutex work_mutex_;
    std::condition_variable work_cv_;
    size_t work_pending_{0};
    std::deque<TrackedFeatureFrame> feature_queue_;
    std::atomic<bool> running_{true};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
