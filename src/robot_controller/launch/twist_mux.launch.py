import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

	# Declare the launch arguments
	use_sim_time = LaunchConfiguration("use_sim_time")
	declare_use_sim_time = DeclareLaunchArgument(
		name="use_sim_time",
		default_value="false",
		description="Use simulation (Gazebo) clock if true",
	)

	twist_mux_node = Node(
		condition=UnlessCondition(use_sim_time),
		package="twist_mux",
		executable="twist_mux",
		parameters=[os.path.join(get_package_share_directory("robot_controller"), "config", "twist_mux.yaml")],
		remappings=[("/cmd_vel_out", "/wheel_controller/cmd_vel_unstamped")],
	)

	twist_mux_gazebo_node = Node(
		condition=IfCondition(use_sim_time),
		package="twist_mux",
		executable="twist_mux",
		parameters=[os.path.join(get_package_share_directory("robot_controller"), "config", "twist_mux.yaml")],
		remappings=[("/cmd_vel_out", "/cmd_vel")],
	)

	return LaunchDescription([
		declare_use_sim_time,
		twist_mux_node,
		twist_mux_gazebo_node,
	])