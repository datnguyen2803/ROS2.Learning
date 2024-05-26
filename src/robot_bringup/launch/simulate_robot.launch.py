import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

	# some variables
	rsp_node_name = 'robot_state_publisher_node'
	is_simulate = "true"

	# packages path
	package_path_robot_controller = get_package_share_directory("robot_controller")
	package_path_robot_navigation = get_package_share_directory("robot_navigation")

	is_scanning_map = LaunchConfiguration("is_scanning_map") # true or false
	declare_is_scanning_map = DeclareLaunchArgument(
		name="is_scanning_map",
		default_value="true",
		description="Run slam toolbox and disable localization if true",
	)


	# run command xacro ... to generate a string
	robot_state_publisher_params = ParameterValue(
		Command(
			[
				'xacro ',
				os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro"),
				' is_simulate:=', is_simulate,
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

	run_rviz_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_description"),
			"launch",
			"sim_display.launch.py"
		),
	)

	run_gazebo_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_description"),
			"launch",
			"sim_gazebo.launch.py"
		),
		condition=IfCondition(is_simulate),
	)

	run_controllers_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_controller"),
			"launch",
			"controller.launch.py"
		),
		launch_arguments={
			'use_sim_time': is_simulate,
			'rsp_node_name': rsp_node_name,
		}.items()
	)

	run_joystick_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_controller"),
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
		launch_arguments={
			'use_sim_time': is_simulate,
		}.items()
	)

	run_navigation_stack_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_navigation"),
			"launch",
			"navigation.launch.py"
		),
		launch_arguments={
			'use_sim_time': 'true',
			}.items()
	)

	run_localization_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_navigation,
			"launch",
			"localization.launch.py"
		),
		condition=UnlessCondition(is_scanning_map),
		launch_arguments={
			'use_sim_time': is_simulate,
		}.items()
	)

	run_slam_toolbox_cmd = IncludeLaunchDescription(
		os.path.join(
			package_path_robot_navigation,
			"launch",
			"slam_online_async.launch.py"
		),
		condition=IfCondition(is_scanning_map),
		launch_arguments={
			'use_sim_time': is_simulate,
		}.items()
	)

	return LaunchDescription([
		declare_is_scanning_map,
		start_robot_state_publisher_cmd,
		run_rviz_cmd,
		run_gazebo_cmd,
		run_controllers_cmd,
		run_joystick_cmd,
		run_twist_mux_cmd,
		# run_navigation_stack_cmd,
		# run_slam_toolbox_cmd,
		# run_localization_cmd,
	])