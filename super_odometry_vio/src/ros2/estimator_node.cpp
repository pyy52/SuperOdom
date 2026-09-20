// ROS2 adapter for the vendored VINS sliding-window estimator, extended in
// Phase 3 with the LiDAR depth associator boundary:
//   LiDAR scans -> bounded buffer -> image-space association at each tracked
//   feature frame -> ExternalFeatureDepth[] -> wrapper seed (gate sections
//   16/17), plus coverage/rejection statistics and the debug overlay.
// All estimator access stays in one worker thread; image callbacks never wait
// for LiDAR (bounded, causal, gate section 29).
#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <thread>

#include <camodocal/camera_models/CameraFactory.h>
#include <opencv2/opencv.hpp>

#include "super_odometry_vio/lidar_depth/lidar_depth_associator.hpp"
#include "super_odometry_vio/vio_estimator.hpp"

using namespace super_odometry_vio;

namespace {

int64_t stampToNs(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<int64_t>(stamp.sec) * 1000000000ll + stamp.nanosec;
}

Sophus::SE3d se3FromArray(const std::vector<double>& a)
{
    // Row-major 4x4 (launch params document the matrix row by row).
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            m(r, c) = a[static_cast<size_t>(r * 4 + c)];
    return Sophus::SE3d(Eigen::Matrix3d(m.block<3, 3>(0, 0)),
                        Eigen::Vector3d(m.block<3, 1>(0, 3)));
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

        if (!wcfg.trajectory_csv.empty())
            std::ofstream(wcfg.trajectory_csv, std::ios::trunc);

        const std::string imu_topic =
            this->declare_parameter<std::string>("imu_topic", wrapper_.imuTopic());
        const std::string feature_topic = this->declare_parameter<std::string>(
            "feature_topic", "/vio_feature_tracker_node/feature");

        // ---------------- Phase 3: lidar_depth ----------------
        lidar_depth::LidarDepthConfig dcfg;
        dcfg.enable = this->declare_parameter<bool>("lidar_depth.enable", false);
        dcfg.max_scans = this->declare_parameter<int>("lidar_depth.max_scans", 3);
        dcfg.max_age_sec = this->declare_parameter<double>("lidar_depth.max_age_sec", 0.12);
        dcfg.min_depth_m = this->declare_parameter<double>("lidar_depth.min_depth_m", 0.3);
        dcfg.max_depth_m = this->declare_parameter<double>("lidar_depth.max_depth_m", 30.0);
        dcfg.pixel_search_radius =
            this->declare_parameter<double>("lidar_depth.pixel_search_radius", 4.0);
        dcfg.min_support_points =
            this->declare_parameter<int>("lidar_depth.min_support_points", 3);
        dcfg.max_depth_spread_m =
            this->declare_parameter<double>("lidar_depth.max_depth_spread_m", 0.3);
        dcfg.max_relative_depth_spread =
            this->declare_parameter<double>("lidar_depth.max_relative_depth_spread", 0.10);
        if (dcfg.enable)
        {
            const std::string camera_config_file = this->declare_parameter<std::string>(
                "lidar_depth.camera_config_file", "");
            const std::string lidar_topic =
                this->declare_parameter<std::string>("lidar_depth.lidar_topic", "");
            dcfg.image_width = this->declare_parameter<int>("lidar_depth.image_width", 0);
            dcfg.image_height = this->declare_parameter<int>("lidar_depth.image_height", 0);
            overlay_dir_ = this->declare_parameter<std::string>(
                "lidar_depth.debug_overlay_dir", "");
            overlay_every_ = this->declare_parameter<int>("lidar_depth.debug_overlay_every", 40);

            const std::vector<double> T_B_L_v = this->declare_parameter<std::vector<double>>(
                "lidar_depth.T_B_L",
                std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
            // T_B_C comes from the estimator settings (RIC/TIC globals).
            const Sophus::SE3d T_B_C(Eigen::Quaterniond(RIC[0]).normalized(), TIC[0]);
            associator_ = std::make_unique<lidar_depth::LidarDepthAssociator>(
                camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
                    camera_config_file),
                dcfg, se3FromArray(T_B_L_v), T_B_C);

            sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar_topic, 2,
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
                {
                    lidar_depth::RecentLidarScan scan;
                    scan.stamp_ns = stampToNs(msg->header.stamp);
                    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"),
                        iy(*msg, "y"), iz(*msg, "z");
                    scan.points_lidar.reserve(msg->width * msg->height);
                    for (; ix != ix.end(); ++ix, ++iy, ++iz)
                        scan.points_lidar.emplace_back(*ix, *iy, *iz);
                    std::lock_guard<std::mutex> lk(assoc_mutex_);
                    associator_->pushScan(std::move(scan));
                });

            if (!overlay_dir_.empty())
            {
                sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
                    this->declare_parameter<std::string>("lidar_depth.image_topic",
                                                         "/cam0/image_raw"),
                    2,
                    [this](const sensor_msgs::msg::Image::SharedPtr msg)
                    {
                        cv_bridge::CvImageConstPtr cv =
                            cv_bridge::toCvCopy(msg, "bgr8");
                        std::lock_guard<std::mutex> lk(assoc_mutex_);
                        last_image_ = cv->image.clone();
                        last_image_stamp_ns_ = stampToNs(msg->header.stamp);
                    });
            }
            pub_depth_stats_ = this->create_publisher<std_msgs::msg::String>(
                "~/vio_depth_stats", 5);
            const auto& T = associator_->T_C_L();
            Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
            m.block<3, 3>(0, 0) = T.rotationMatrix();
            m.block<3, 1>(0, 3) = T.translation();
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6) << m;
            RCLCPP_INFO(this->get_logger(), "lidar_depth ENABLED (lidar: %s)\nT_C_L=\n%s",
                        lidar_topic.c_str(), oss.str().c_str());
        }

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 200,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                ImuSample s;
                s.stamp_ns = stampToNs(msg->header.stamp);
                s.accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
                    msg->linear_acceleration.z;
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
                work_cv_.wait(lk,
                              [this]() { return work_pending_ > 0 || !running_; });
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
                if (associator_) runDepthAssociation(frame);
                wrapper_.processFeatures(frame);
                publishOutputs();
            }
            {
                std::lock_guard<std::mutex> lk(work_mutex_);
                work_pending_ = 0;
            }
        }
    }

    // Phase 3 boundary: tracked features -> depth association -> external
    // depths for the wrapper seed. Bounded, causal, never blocks the VIO.
    void runDepthAssociation(const TrackedFeatureFrame& frame)
    {
        lidar_depth::LidarDepthFrameResult result;
        {
            std::lock_guard<std::mutex> lk(assoc_mutex_);
            result = associator_->associate(frame.observations, frame.stamp_ns);
        }

        wrapper_.setExternalDepths(result.depths);

        // Accumulate coverage/rejection statistics (gate section 24).
        total_.tracked += result.stats.tracked_features;
        total_.candidates += result.stats.features_with_candidates;
        total_.accepted += result.stats.features_with_accepted_depth;
        total_.projected += result.stats.projected_lidar_points;
        for (const auto& kv : result.stats.rejects)
            total_rejects_[kv.first] += kv.second;
        if (++frames_since_stats_ >= 20)
        {
            char buf[512];
            snprintf(buf, sizeof(buf),
                     "{\"tracked\":%d,\"candidates\":%d,\"accepted\":%d,"
                     "\"depth_supported_ratio\":%.3f,\"projected_pts\":%d,"
                     "\"rejects\":{%s}}",
                     total_.tracked, total_.candidates, total_.accepted,
                     total_.tracked > 0
                         ? static_cast<double>(total_.accepted) / total_.tracked
                         : 0.0,
                     total_.projected, rejectSummary().c_str());
            std_msgs::msg::String s;
            s.data = buf;
            pub_depth_stats_->publish(s);
            frames_since_stats_ = 0;
        }

        // Debug overlay: raw image + projected LiDAR (gray), tracked (blue),
        // accepted depth (green), rejected (red). Gate section 21.
        if (!overlay_dir_.empty())
        {
            cv::Mat img;
            {
                std::lock_guard<std::mutex> lk(assoc_mutex_);
                if (last_image_stamp_ns_ == frame.stamp_ns) img = last_image_.clone();
            }
            if (!img.empty())
            {
                cv::Mat out;
                cv::cvtColor(img, out, cv::COLOR_BGR2GRAY);
                cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
                {
                    std::lock_guard<std::mutex> lk(assoc_mutex_);
                    for (const auto& p : associator_->lastProjectedPoints())
                        cv::circle(out, cv::Point(static_cast<int>(p.x()),
                                                  static_cast<int>(p.y())),
                                   1, cv::Scalar(160, 160, 160), -1);
                }
                for (const auto& obs : frame.observations)
                {
                    const cv::Point c(static_cast<int>(obs.pixel.x()),
                                      static_cast<int>(obs.pixel.y()));
                    const bool accepted =
                        result.depths.end() !=
                        std::find_if(result.depths.begin(), result.depths.end(),
                                     [&obs](const lidar_depth::ExternalFeatureDepth& d)
                                     { return d.feature_id == obs.feature_id; });
                    if (accepted)
                        cv::circle(out, c, 3, cv::Scalar(0, 255, 0), 1);
                    else if (result.rejected.count(obs.feature_id))
                        cv::circle(out, c, 3, cv::Scalar(0, 0, 255), 1);
                    else
                        cv::circle(out, c, 2, cv::Scalar(255, 128, 0), 1);
                }
                if (++overlay_count_ % overlay_every_ == 0)
                {
                    const std::string path = overlay_dir_ + "/overlay_" +
                                             std::to_string(frame.stamp_ns / 1000000) +
                                             ".png";
                    cv::imwrite(path, out);
                }
            }
        }
    }

    std::string rejectSummary() const
    {
        std::string out;
        for (const auto& kv : total_rejects_)
        {
            if (!out.empty()) out += ",";
            out += std::string(lidar_depth::toCString(kv.first)) + ":" +
                   std::to_string(kv.second);
        }
        return out;
    }

    void publishOutputs()
    {
        const VioState st = wrapper_.latestState();
        const VioQuality q = wrapper_.latestQuality();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = rclcpp::Time(st.stamp_ns);
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
                 "\"parallax_px\":%.2f,\"solve_p50_ms\":%.2f,\"solve_p95_ms\":%.2f,"
                 "\"info_min_eig\":%.4g,\"info_cond\":%.4g,\"observable_dof\":%d,"
                 "\"optimizer\":%s}",
                 static_cast<double>(st.stamp_ns) * 1e-9, toCString(q.health),
                 q.tracked_features, q.inlier_features, q.median_parallax_px,
                 q.solve_time_ms_p50, q.solve_time_ms_p95,
                 q.information_min_eigenvalue, q.information_condition_number,
                 q.observable_dof, q.optimizer_success ? "true" : "false");
        std_msgs::msg::String health;
        health.data = buf;
        pub_health_->publish(health);
    }

    VioEstimator wrapper_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_feature_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_health_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_depth_stats_;

    // Phase 3 lidar depth
    std::unique_ptr<lidar_depth::LidarDepthAssociator> associator_;
    std::mutex assoc_mutex_;
    cv::Mat last_image_;
    int64_t last_image_stamp_ns_{0};
    std::string overlay_dir_;
    int overlay_every_{40};
    int overlay_count_{0};
    struct
    {
        int tracked{0};
        int candidates{0};
        int accepted{0};
        int projected{0};
    } total_;
    std::map<lidar_depth::RejectReason, int> total_rejects_;
    int frames_since_stats_{0};

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
