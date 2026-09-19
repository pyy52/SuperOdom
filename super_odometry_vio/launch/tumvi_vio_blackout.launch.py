import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("super_odometry_vio")
    tracker_cfg = os.path.join(share, "config", "tum_vi_room1.yaml")
    estimator_cfg = os.path.join(share, "config", "tum_vi_room1_vio.yaml")
    trajectory_csv = os.environ.get("VIO_TRAJECTORY_CSV", "/tmp/vio_blackout.csv")
    # Intervals relative to the first seen image stamp: "start:duration,..."
    blackout_intervals = os.environ.get("VIO_BLACKOUT_INTERVALS", "40:15")

    blackout_node = Node(
        package="super_odometry_vio",
        executable="camera_blackout_node",
        output="screen",
        parameters=[{
            "in_topic": "/cam0/image_raw",
            "out_topic": "/cam0/image_blackout",
            "intervals": blackout_intervals,
        }],
    )

    tracker_node = Node(
        package="super_odometry_vio",
        executable="vio_feature_tracker_node",
        output="screen",
        parameters=[{
            "config_file": tracker_cfg,
            "image_topic": "/cam0/image_blackout",
        }],
    )

    estimator_node = Node(
        package="super_odometry_vio",
        executable="vio_estimator_node",
        output="screen",
        parameters=[{
            "config_file": estimator_cfg,
            "trajectory_csv": trajectory_csv,
            "feature_topic": "/vio_feature_tracker_node/feature",
        }],
    )

    return LaunchDescription([blackout_node, tracker_node, estimator_node])
