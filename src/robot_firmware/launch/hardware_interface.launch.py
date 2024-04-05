import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

	robot_description = ParameterValue(
		Command(
			[
				# "xacro ",
				# os.path.join(
				# 	get_package_share_directory("bumperbot_description"),
				# 	"urdf",
				# 	"bumperbot.urdf.xacro",
				# ),
				# " is_sim:=False"
			]
		),
		value_type=str,
	)

	controller_manager_node = Node(
		package='controller_manager',
		executable='ros2_control_node',
		parameters=[
			# {"robot_description": robot_description,
			# "use_sim_time": False},
			os.path.join(
				get_package_share_directory('robot_controller'),
				"config",
				"robot_controllers.yaml"
			)
		]
	)


	return LaunchDescription([
		controller_manager_node
	])


