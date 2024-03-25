#ifndef __SIMPLE__TURTLESIM__KINEMATICS__HPP__
#define __SIMPLE__TURTLESIM__KINEMATICS__HPP__

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class SimpleTurtleSimKinematics : public rclcpp::Node
{
public:
	SimpleTurtleSimKinematics(const std::string& node_name);

private:
	void turtle1PoseCallback(const turtlesim::msg::Pose &pose);

	void turtle2PoseCallback(const turtlesim::msg::Pose &pose);

	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;

	turtlesim::msg::Pose last_turtle1_pose_;
	turtlesim::msg::Pose last_turtle2_pose_;

};


#endif