import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

	package_path_robot_navigation = get_package_share_directory("robot_navigation")

	run_slam_toolbox_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_navigation,
			"launch",
			"slam_online_async.launch.py"
		),
		launch_arguments={
			'use_sim_time': "false",
		}.items()
	)

	run_navigation_stack_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_navigation,
			"launch",
			"navigation.launch.py"
		),
		launch_arguments={
			'use_sim_time': "false",
		}.items()
	)

	run_localization_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_navigation,
			"launch",
			"localization.launch.py"
		),
		launch_arguments={
			'use_sim_time': "false",
		}.items()
	)

	return LaunchDescription([
		run_slam_toolbox_cmd,
		run_navigation_stack_cmd,
		# run_localization_cmd,

	])