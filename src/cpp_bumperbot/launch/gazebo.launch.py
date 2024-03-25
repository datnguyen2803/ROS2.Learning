from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,  IncludeLaunchDescription
import os
from  os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

	bumperbot_output_path  = get_package_share_directory("cpp_bumperbot")
	bumperbot_prefix = get_package_prefix("cpp_bumperbot")

	models_path = os.path.join(bumperbot_output_path, "models")
	models_path += pathsep + os.path.join(bumperbot_prefix, "share")

	env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", models_path)

	model_arg = DeclareLaunchArgument(
		name="model",
		default_value=os.path.join(bumperbot_output_path, "urdf", "bumperbot.urdf.xacro"),
		description="Absolute path to robot urdf file",
	)

	robot_description = ParameterValue(Command([" xacro ", LaunchConfiguration("model")]), value_type=str)
	
	robot_state_publisher_node = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		parameters=[
			{"robot_description": robot_description},
		],
	)

	
	# start gazebo server and client from package gazebo_ros, dir launch. needed to use gazebo
	# server: simulates objects, environment and calculates physics, interactions
	# client: GUI to visualize simulation
	start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")))
	start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")))

	spawn_robot = Node(
		package="gazebo_ros",
		executable="spawn_entity.py",
		arguments=["-entity", "bumperbot", "-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0"],
		output="screen",

	)

	return LaunchDescription([
		env_variable,
		model_arg,
		robot_state_publisher_node,
		start_gazebo_server,
		start_gazebo_client,
		spawn_robot,
	])
