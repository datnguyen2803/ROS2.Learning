#ifndef __SIMPLE_CONTROLLER_HPP__
#define __SIMPLE_CONTROLLER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>

class SimpleController : public rclcpp::Node
{
public:
	SimpleController(const std::string& node_name);

private:
	void velCallback(const geometry_msgs::msg::TwistStamped &msg);
	void jointCallback(const sensor_msgs::msg::JointState &msg);

	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	

	double wheel_radius_;
	double wheel_separation_;
	Eigen::Matrix2d speed_conversion_;

	double left_wheel_prev_pos_;
	double right_wheel_prev_pos_;
	rclcpp::Time prev_time_;

	double x_;
	double y_;
	double theta_;

	nav_msgs::msg::Odometry odom_msg_;

	geometry_msgs::msg::TransformStamped odom_tf_msg_;


};

#endif // __SIMPLE_CONTROLLER_HPP__