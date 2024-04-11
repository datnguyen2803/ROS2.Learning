import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

	model_arg = DeclareLaunchArgument(
		name="model",
		default_value=os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro"),
		description="Absolute path to robot urdf file",
	)

	model_path = os.path.join(get_package_share_directory("robot_description"), "models")
	model_path += pathsep + os.path.join(get_package_prefix("robot_description"), "share")
	# model_path = os.path.join(get_package_prefix("robot_description"), "share")

	env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

	world = os.path.join(
		get_package_share_directory('robot_navigation'),
		'worlds',
		'scan_world.world'
	)

	# urdf_path = os.path.join(
	# 	get_package_share_directory('turtlebot3_gazebo'),
	# 	'models',
	# 	'turtlebot3_waffle',
	# 	'model.sdf'
    # )

	start_gazebo_server_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('gazebo_ros'),
				'launch',
				'gzserver.launch.py'
			)
		),
		launch_arguments={'world': world}.items()
	)

	start_gazebo_client_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('gazebo_ros'),
				'launch',
				'gzclient.launch.py'
			)
		)
	)

	spawn_robot_cmd = Node(
		package="gazebo_ros",
		executable="spawn_entity.py",
		name="gazebo_node",
		arguments=[ #required arguments must be passed
			"-entity", "mikey_robot",
			"-topic", "robot_description",
		# 	"-file", urdf_path,
			'-x', '0.00',
			'-y', '0.00',
			'-z', '0.00',
		],
		output="screen"
	)

	return LaunchDescription([
		env_var,
		model_arg,
		start_gazebo_server_cmd,
		start_gazebo_client_cmd,
		spawn_robot_cmd,
	])