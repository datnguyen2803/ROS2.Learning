import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

	# some variables
	rsp_node_name = 'robot_state_publisher_node'

	# packages path
	package_path_robot_controller = get_package_share_directory("robot_controller")
	package_path_robot_navigation = get_package_share_directory("robot_navigation")

	is_simulate = LaunchConfiguration("is_simulate") # yes or no

	# Declare the launch arguments
	declare_is_simulate = DeclareLaunchArgument(
		name="is_simulate",
		default_value="no",
		description="Use simulation (Gazebo) clock if yes",
	)

	# run command xacro ... to generate a string
	robot_state_publisher_params = ParameterValue(
		Command(
			[
				'xacro ',
				os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro"),
				' is_simulate:=', "true" if is_simulate == "yes" else "false",
			]
		), 
		value_type=str
	)

	# start robot state publisher rsp
	start_robot_state_publisher_cmd = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name=rsp_node_name,
		parameters=[{'robot_description': robot_state_publisher_params}],
	)

	run_controllers_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_controller,
			"launch",
			"controller.launch.py"
		),
		launch_arguments={
			'use_sim_time': "true" if is_simulate == "yes" else "false",
			'rsp_node_name': rsp_node_name,
		}.items()
	)

	start_controllers_delayed_cmd = RegisterEventHandler(
		# condition=UnlessCondition(use_sim_time),
		event_handler=OnProcessStart(
			target_action=start_robot_state_publisher_cmd,
			on_start=[run_controllers_cmd]
		)
	)

	run_joystick_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_controller,
			"launch",
			"joystick_teleop.launch.py"
		),
	)

	run_twist_mux_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_controller,
			"launch",
			"twist_mux.launch.py"
		),
		# launch_arguments={
		# 	'use_sim_time': 'false' if is_simulate == "no" else 'true',
		# }.items()
	)

	run_lidar_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_controller,
			"launch",
			"lidar.launch.py"
		),
	)

	run_navigation_stack_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_navigation,
			"launch",
			"navigation.launch.py"
		),
		launch_arguments={
			'use_sim_time': "true" if is_simulate == "yes" else "false",
		}.items()
	)

	run_localization_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_navigation,
			"launch",
			"localization.launch.py"
		),
		launch_arguments={
			'use_sim_time': "true" if is_simulate == "yes" else "false",
		}.items()
	)

	run_slam_toolbox_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_navigation,
			"launch",
			"slam_online_async.launch.py"
		),
		launch_arguments={
			'use_sim_time': "true" if is_simulate == "yes" else "false",
		}.items()
	)

	run_camera_cmd = Node(
		package='v4l2_camera',
		executable='v4l2_camera_node',
		output='screen',
		namespace='camera',
		parameters=[{
			'image_size': [640, 480],
			'time_per_frame': [1, 6],
			'camera_frame_id': 'camera_link_optical'
			}]
	)

	return LaunchDescription([
		declare_is_simulate,
		start_robot_state_publisher_cmd,
		start_controllers_delayed_cmd,
		# run_joystick_cmd,
		# run_twist_mux_cmd,
		# run_lidar_cmd,
		# run_slam_toolbox_cmd,
		# run_navigation_stack_cmd,
		# run_localization_cmd,

		# run_camera_cmd,

	])