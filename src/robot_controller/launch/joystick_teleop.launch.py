import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

	# Joystick driver
	# Read joystick command and publish to /<topic_xxx> in ROS2
	joy_node = Node(
		package='joy',
		executable='joy_node',
		name='joystick',
		parameters=[os.path.join(get_package_share_directory('robot_controller'), 'config', 'joystick_config.yaml')],
	)

	# Joystick teleop
	# Subscribe to /<topic_xxx> and publish velocity mesage for simple controller
	joy_teleop = Node(
		package="joy_teleop",
		executable="joy_teleop",
		name="joy_teleop",
		parameters=[os.path.join(get_package_share_directory('robot_controller'), 'config', 'joystick_teleop.yaml')],
	)

	return LaunchDescription([
		joy_node,
		joy_teleop,
	])