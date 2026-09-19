import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("super_odometry_vio"),
        "config", "tum_vi_room1.yaml")

    tracker_node = Node(
        package="super_odometry_vio",
        executable="vio_feature_tracker_node",
        output="screen",
        parameters=[{
            "config_file": config_path,
            # Empty string -> fall back to image_topic from the settings file.
            "image_topic": "",
        }],
    )

    return LaunchDescription([tracker_node])
