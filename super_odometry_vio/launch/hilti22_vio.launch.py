import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("super_odometry_vio")
    tracker_cfg = os.path.join(share, "config", "hilti22_exp14_tracker.yaml")
    estimator_cfg = os.path.join(share, "config", "hilti22_exp14_vio.yaml")
    trajectory_csv = os.environ.get(
        "VIO_TRAJECTORY_CSV", "/tmp/vio_trajectory_hilti22.csv")
    depth_enable = os.environ.get("VIO_LIDAR_DEPTH_ENABLE", "false").lower() == "true"
    overlay_dir = os.environ.get("VIO_DEPTH_OVERLAY_DIR", "")

    # HILTI22 official T_SL (IMU<-LiDAR) from OKVIS2-X config, row-major 4x4.
    T_B_L = [0.0, -1.0, 0.0, -0.0007,
             -1.0, 0.0, 0.0, -0.0086,
             0.0, 0.0, -1.0, 0.0550,
             0.0, 0.0, 0.0, 1.0]

    tracker_node = Node(
        package="super_odometry_vio",
        executable="vio_feature_tracker_node",
        output="screen",
        parameters=[{
            "config_file": tracker_cfg,
            "image_topic": "",
        }],
    )

    estimator_params = [{
        "config_file": estimator_cfg,
        "trajectory_csv": trajectory_csv,
        "feature_topic": "/vio_feature_tracker_node/feature",
        "lidar_depth.enable": depth_enable,
        "lidar_depth.lidar_topic": "/hesai/pandar",
        "lidar_depth.camera_config_file": tracker_cfg,
        "lidar_depth.image_width": 720,
        "lidar_depth.image_height": 540,
        "lidar_depth.image_topic": "/alphasense/cam0/image_raw",
        "lidar_depth.T_B_L": T_B_L,
        "lidar_depth.max_scans": 3,
        "lidar_depth.time_mode": os.environ.get("VIO_LIDAR_TIME_MODE", "scan_stamp")  # tiers: scan_stamp | point_rotation,
        "lidar_depth.max_age_sec": 0.12,
        "lidar_depth.debug_overlay_dir": overlay_dir,
        "lidar_depth.debug_overlay_every": 60,
    }]
    if not depth_enable:
        # keep the params minimal but present for the A/B run
        pass

    estimator_node = Node(
        package="super_odometry_vio",
        executable="vio_estimator_node",
        output="screen",
        parameters=estimator_params,
    )

    return LaunchDescription([tracker_node, estimator_node])
