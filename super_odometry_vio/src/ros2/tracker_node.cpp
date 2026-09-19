// ROS2 adapter for the ported VINS-Mono feature-tracking core.
// Mirrors upstream feature_tracker_node.cpp behaviour (rate gating, ID
// assignment, PointCloud feature publishing with the same channel order the
// VINS estimator expects: 0=id, 1=u, 2=v, 3=velocity_x, 4=velocity_y), so the
// estimator core ported next consumes an unchanged stream format.
#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <string>

#include "feature_tracker.h"
#include "parameters.h"
#include "tracker_log.h"

class VioFeatureTrackerNode : public rclcpp::Node
{
  public:
    VioFeatureTrackerNode() : Node("vio_feature_tracker_node"), pub_count_(0)
    {
        config_file_ = this->declare_parameter<std::string>("config_file", "");
        if (config_file_.empty())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "parameter 'config_file' (VINS-style settings yaml) is required");
            throw std::runtime_error("config_file not set");
        }
        image_topic_ = this->declare_parameter<std::string>("image_topic", "");

        // Fisheye mask path: explicit param, else fisheye_mask.jpg next to
        // the settings file (installed alongside it, as in upstream).
        const std::string mask_param =
            this->declare_parameter<std::string>("fisheye_mask", "");
        readParameters(config_file_, mask_param);
        tracker_.readIntrinsicParameter(CAM_NAMES.front());
        if (FISHEYE == 1)
        {
            std::string mask_path = FISHEYE_MASK;
            if (mask_path.empty())
            {
                const size_t slash = config_file_.find_last_of('/');
                mask_path = config_file_.substr(0, slash + 1) + "fisheye_mask.jpg";
            }
            tracker_.fisheye_mask = cv::imread(mask_path, 0);
            if (tracker_.fisheye_mask.empty())
                RCLCPP_WARN(this->get_logger(),
                            "fisheye mask '%s' not loadable; continuing without",
                            mask_path.c_str());
        }

        // Config topics win unless the launch file overrides them explicitly.
        if (image_topic_.empty())
            image_topic_ = IMAGE_TOPIC;

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 2,
            std::bind(&VioFeatureTrackerNode::imageCallback, this, std::placeholders::_1));
        pub_feature_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
            "~/feature", 5);

        first_image_time_ = -1.0;
        RCLCPP_INFO(this->get_logger(), "vio feature tracker ready: %s <- %s",
                    (this->get_name() + std::string("/feature")).c_str(),
                    image_topic_.c_str());
    }

  private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        const double t = rclcpp::Time(img_msg->header.stamp).seconds();
        if (first_image_time_ < 0.0)
            first_image_time_ = t;

        // Upstream rate gate: publish features at ~FREQ Hz.
        if (FREQ != 0 &&
            std::round(1.0 * pub_count_ / (t - first_image_time_)) <= FREQ)
        {
            PUB_THIS_FRAME = true;
            if (std::abs(1.0 * pub_count_ / (t - first_image_time_) - FREQ) <
                0.01 * FREQ)
            {
                first_image_time_ = t;
                pub_count_ = 0;
            }
        }
        else if (FREQ == 0)
        {
            PUB_THIS_FRAME = true;
        }
        else
        {
            PUB_THIS_FRAME = false;
        }

        cv_bridge::CvImageConstPtr ptr =
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        tracker_.readImage(ptr->image.rowRange(0, ROW).colRange(0, COL), t);

        // Node owns ID assignment for new (-1) tracks, exactly like upstream.
        for (unsigned int i = 0;; ++i)
        {
            if (!tracker_.updateID(i))
                break;
        }

        if (!PUB_THIS_FRAME)
            return;
        ++pub_count_;

        sensor_msgs::msg::PointCloud feature_points;
        feature_points.header = img_msg->header;
        feature_points.header.frame_id = "world";
        sensor_msgs::msg::ChannelFloat32 id_of_point, u_of_point, v_of_point,
            velocity_x_of_point, velocity_y_of_point;

        for (unsigned int j = 0; j < tracker_.ids.size(); ++j)
        {
            if (tracker_.track_cnt[j] <= 1)
                continue;  // skip the first appearance: no velocity yet
            geometry_msgs::msg::Point32 p;
            p.x = tracker_.cur_un_pts[j].x;
            p.y = tracker_.cur_un_pts[j].y;
            p.z = 1.0f;
            feature_points.points.push_back(p);
            id_of_point.values.push_back(static_cast<float>(tracker_.ids[j]));
            u_of_point.values.push_back(tracker_.cur_pts[j].x);
            v_of_point.values.push_back(tracker_.cur_pts[j].y);
            velocity_x_of_point.values.push_back(tracker_.pts_velocity[j].x);
            velocity_y_of_point.values.push_back(tracker_.pts_velocity[j].y);
        }
        feature_points.channels.push_back(id_of_point);
        feature_points.channels.push_back(u_of_point);
        feature_points.channels.push_back(v_of_point);
        feature_points.channels.push_back(velocity_x_of_point);
        feature_points.channels.push_back(velocity_y_of_point);

        pub_feature_->publish(feature_points);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_feature_;

    FeatureTracker tracker_;
    std::string config_file_;
    std::string image_topic_;
    double first_image_time_;
    int pub_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioFeatureTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
