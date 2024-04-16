import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

	twist_mux_node = Node(
		package="twist_mux",
		executable="twist_mux",
		parameters=[os.path.join(get_package_share_directory("robot_controller"), "config", "twist_mux.yaml")],
		remappings=[("/cmd_vel_out", "/wheel_controller/cmd_vel_unstamped")],
	)

	return LaunchDescription([
		twist_mux_node,
	])