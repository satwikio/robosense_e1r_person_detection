"""Step 2: E1R driver + person detector + RViz.

  ros2 launch e1r_person_detection e1r_person_detection.launch.py               # real lidar
  ros2 launch e1r_person_detection e1r_person_detection.launch.py fake:=true    # synthetic scene
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("e1r_person_detection")
    fake = LaunchConfiguration("fake")
    return LaunchDescription([
        DeclareLaunchArgument("fake", default_value="false", description="Use the synthetic lidar instead of the E1R"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("lidar_config", default_value=os.path.join(share, "config", "rslidar_e1r.yaml")),
        DeclareLaunchArgument("detector_config", default_value=os.path.join(share, "config", "detector.yaml")),
        Node(package="rslidar_sdk", executable="rslidar_sdk_node", name="rslidar_sdk_node", output="screen",
             parameters=[{"config_path": LaunchConfiguration("lidar_config")}], condition=UnlessCondition(fake)),
        Node(package="e1r_person_detection", executable="fake_lidar", name="fake_e1r", output="screen",
             condition=IfCondition(fake)),
        Node(package="e1r_person_detection", executable="person_detector", name="person_detector", output="screen",
             parameters=[LaunchConfiguration("detector_config")]),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "e1r.rviz")], condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
