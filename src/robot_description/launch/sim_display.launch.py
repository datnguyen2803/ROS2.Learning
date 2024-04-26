from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	use_sim_time_arg = DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='Use simulation (Gazebo) clock if true')


	# model=os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro")


	rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'),
									'rviz', 'nav2_default_view.rviz')

	# robot_description = Command(['xacro ', model, ' is_simulate:=false'])

	# robot_state_publisher_node = Node(
	# 	package="robot_state_publisher",
	# 	executable="robot_state_publisher",
	# 	name="robot_state_publisher",
	# 	parameters=[{"robot_description": robot_description}],
	# )

	# ui to control joints, must have
	joint_state_publisher_gui = Node(
		package="joint_state_publisher_gui",
		executable="joint_state_publisher_gui",

	)

	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="screen",
		arguments=["-d", rviz_config_dir],
		parameters=[{
			"use_sim_time": use_sim_time,
		}],

	)

	return LaunchDescription([
		use_sim_time_arg,
		# robot_state_publisher_node,
		joint_state_publisher_gui,
		rviz_node,
	])