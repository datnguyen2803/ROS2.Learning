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

	# Declare the launch arguments
	use_sim_time = LaunchConfiguration("use_sim_time")
	rsp_node_name = LaunchConfiguration("rsp_node_name")
	
	declare_use_sim_time = DeclareLaunchArgument(
		name="use_sim_time",
		default_value="false",
		description="Use simulation (Gazebo) clock if true",
	)
	declare_rsp_node_name = DeclareLaunchArgument(
		name="rsp_node_name",
		default_value="robot_state_publisher_node",
		description="Name of node robot_state_publisher, used to get robot_description",
	)

	# find files
	controller_manager_params_file = os.path.join(
				get_package_share_directory('robot_controller'),
				"config",
				"robot_controllers.yaml"
			)
	
	robot_description = ParameterValue(Command(['ros2 param get --hide-type /', rsp_node_name, ' robot_description']), value_type=str)

	# create controller manager
	start_controller_manager_cmd = Node(
		# condition=UnlessCondition(use_sim_time),
		package='controller_manager',
		executable='ros2_control_node',
		parameters=[
			{
				'robot_description': robot_description,
				# 'use_sim_time': use_sim_time,
			},
			controller_manager_params_file
		]
	)

	# delay 1s before create controller manager
	start_delayed_controller_manager_cmd = TimerAction(period=5.0, actions=[start_controller_manager_cmd])

	# spawner != ros2_control_node that spawner runs only once but ros2_control_node spins
	start_diff_drive_controller_cmd = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["wheel_controller",
			"--controller-manager", "/controller_manager"
		]
	)
	start_joint_state_broadcaster_cmd = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["joint_state_broadcaster",
			"--controller-manager", "/controller_manager"
		]
	)
	start_imu_broadcaster_cmd = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["imu_broadcaster",
			"--controller-manager", "/controller_manager"
		]
	)

	# load controllers until controller manager is ready
	start_delayed_diff_drive_controller_cmd = RegisterEventHandler(
		# condition=UnlessCondition(use_sim_time),
		event_handler=OnProcessStart(
			target_action=start_controller_manager_cmd,
			on_start=[start_diff_drive_controller_cmd]
		)
	)

	start_delayed_joint_state_broadcaster_cmd = RegisterEventHandler(
		# condition=UnlessCondition(use_sim_time),
		event_handler=OnProcessStart(
			target_action=start_controller_manager_cmd,
			on_start=[start_joint_state_broadcaster_cmd]
		)
	)

	start_delayed_imu_broadcaster_cmd = RegisterEventHandler(
		# condition=UnlessCondition(use_sim_time),
		event_handler=OnProcessStart(
			target_action=start_controller_manager_cmd,
			on_start=[start_imu_broadcaster_cmd]
		)
	)

	ekf_params_file = os.path.join(get_package_share_directory('robot_navigation'), "config/ekf.yaml")
		# Start robot localization using an Extended Kalman Filter
	ekf_localization_node = Node(
		package="robot_localization",
		executable="ekf_node",
		parameters=[ekf_params_file],
	)


	return LaunchDescription([
		declare_use_sim_time,
		declare_rsp_node_name,
		start_delayed_controller_manager_cmd,
		start_delayed_diff_drive_controller_cmd,
		start_delayed_joint_state_broadcaster_cmd, 
		start_delayed_imu_broadcaster_cmd,

		ekf_localization_node,

		# start_diff_drive_controller_cmd,

	])
