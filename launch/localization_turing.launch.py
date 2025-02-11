import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # 读取参数
    use_sim_time = LaunchConfiguration("use_sim_time", default=os.getenv("SIMULATION", "false"))
    rviz_config_file = os.path.join(
        get_package_share_directory("funny_lidar_slam"), "launch", "display.rviz"
    )
    param_file = os.path.join(
        get_package_share_directory("funny_lidar_slam"), "config", "localization", "config_turing.yaml"
    )

    # SLAM 节点
    funny_lidar_slam_node = Node(
        package="funny_lidar_slam",
        executable="funny_lidar_slam",
        name="funny_lidar_slam",
        output="screen",
        parameters=[param_file, {"use_sim_time": use_sim_time}],
    )

    # RViz 可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("visualize", default="true")),
    )

    return LaunchDescription([funny_lidar_slam_node, rviz_node])
