import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

	# run command xacro ... to generate a string
	robot_state_publisher_params = ParameterValue(
		Command(
			[
				'xacro ',
				os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro")
			]
		), 
		value_type=str
	)

	# start robot state publisher rsp
	start_robot_state_publisher_cmd = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_node',
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
	)

	run_controllers_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_controller"),
			"launch",
			"controller.launch.py"
		),
	)

	run_joystick_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_controller"),
			"launch",
			"joystick_teleop.launch.py"
		),
	)

	run_navigation_stack_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_navigation"),
			"launch",
			"navigation.launch.py"
		),
		launch_arguments={
			'use_sim_time': 'true',
			# 'params_file': '/home/mikey/mikey_robot/install/robot_navigation/share/robot_navigation/config/nav2_params.yaml',
			}.items()
	)

	# print("mikey done nav2 stack")

	# run_delayed_nav2_bringup_cmd = RegisterEventHandler(
	# 	event_handler=OnProcessStart(
	# 		target_action=run_gazebo_cmd,
	# 		on_start=[run_nav2_bringup_cmd]
	# 	)
	# )
	# run_delayed_nav2_bringup_cmd = TimerAction(period=5.0, actions=[run_navigation_stack_cmd])

	run_slam_toolbox_cmd = IncludeLaunchDescription(
		os.path.join(
			get_package_share_directory("robot_navigation"),
			"launch",
			"slam_online_async.launch.py"
		),
		# launch_arguments={'use_sim_time': 'true'}.items()
	)

	# run_delayed_slam_toolbox_cmd = RegisterEventHandler(
	# 	event_handler=OnProcessStart(
	# 		target_action=run_nav2_bringup_cmd,
	# 		on_start=[run_slam_toolbox_cmd]
	# 	)
	# )
	# run_delayed_slam_toolbox_cmd = TimerAction(period=10.0, actions=[run_slam_toolbox_cmd])

	run_twist_mux_cmd = Node(
		package="twist_mux",
		executable="twist_mux",
		parameters=[os.path.join(get_package_share_directory("robot_controller"), "config", "twist_mux.yaml")],
		remappings=[("/cmd_vel_out", "/wheel_controller/cmd_vel_unstamped")],
	)

	return LaunchDescription([
		start_robot_state_publisher_cmd,
		run_rviz_cmd,
		run_gazebo_cmd,
		run_controllers_cmd,
		run_joystick_cmd,
		run_navigation_stack_cmd,
		run_slam_toolbox_cmd,
		run_twist_mux_cmd,
	])