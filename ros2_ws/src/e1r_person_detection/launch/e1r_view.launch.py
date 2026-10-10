"""Step 1: run the RoboSense E1R driver and show the raw point cloud in RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("e1r_person_detection")
    return LaunchDescription([
        DeclareLaunchArgument("lidar_config", default_value=os.path.join(share, "config", "rslidar_e1r.yaml")),
        Node(package="rslidar_sdk", executable="rslidar_sdk_node", name="rslidar_sdk_node", output="screen",
             parameters=[{"config_path": LaunchConfiguration("lidar_config")}]),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "e1r.rviz")]),
    ])
