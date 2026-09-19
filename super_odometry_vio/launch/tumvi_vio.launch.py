import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("super_odometry_vio")
    tracker_cfg = os.path.join(share, "config", "tum_vi_room1.yaml")
    estimator_cfg = os.path.join(share, "config", "tum_vi_room1_vio.yaml")
    trajectory_csv = os.environ.get("VIO_TRAJECTORY_CSV", "/tmp/vio_trajectory.csv")

    tracker_node = Node(
        package="super_odometry_vio",
        executable="vio_feature_tracker_node",
        output="screen",
        parameters=[{
            "config_file": tracker_cfg,
            "image_topic": "",
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

    return LaunchDescription([tracker_node, estimator_node])
