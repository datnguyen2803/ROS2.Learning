import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

	# find files
	controller_manager_params_file = os.path.join(
				get_package_share_directory('robot_controller'),
				"config",
				"robot_controllers.yaml"
			)
	
	# run command xacro ... to generate a string
	robot_state_publisher_params = Command(
		[
			'xacro ',
			os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro")
		]
	)

	# start robot state publisher rsp
	start_robot_state_publisher_cmd = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_node',
		parameters=[{'robot_description': robot_state_publisher_params}],
	)

	robot_description = Command(['ros2 param get --hide-type /robot_state_publisher_node robot_description'])

	# create controller manager
	start_controller_manager_cmd = Node(
		package='controller_manager',
		executable='ros2_control_node',
		parameters=[
			{'robot_description': robot_description},
			controller_manager_params_file
			
		]
	)
	# delay 2s before create controller manager
	start_delayed_controller_manager_cmd = TimerAction(period=2.0, actions=[start_controller_manager_cmd])

	# spawner != ros2_control_node that spawner runs only once but ros2_control_node spins
	start_diff_drive_controller_cmd = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["wheel_controller"]
	)

	# load diff drive controller until controller manager is ready
	start_delayed_diff_drive_controller_cmd = RegisterEventHandler(
		event_handler=OnProcessStart(
			target_action=start_controller_manager_cmd,
			on_start=[start_diff_drive_controller_cmd]
		)
	)

	return LaunchDescription([
		start_robot_state_publisher_cmd,
		start_delayed_controller_manager_cmd,
		start_delayed_diff_drive_controller_cmd

	])