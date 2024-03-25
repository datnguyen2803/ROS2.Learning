#include "cpp_bumperbot_examples/simple_turtlesim_kinematics.hpp"

using std::placeholders::_1;

SimpleTurtleSimKinematics::SimpleTurtleSimKinematics(const std::string& node_name) : Node(node_name)
{
	turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
		"/turtle1/pose", 10, std::bind(&SimpleTurtleSimKinematics::turtle1PoseCallback, this, _1));
	turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
		"/turtle2/pose", 10, std::bind(&SimpleTurtleSimKinematics::turtle2PoseCallback, this, _1));

}

void SimpleTurtleSimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose &pose)
{
	last_turtle1_pose_ = pose;
}

void SimpleTurtleSimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose &pose)
{
	last_turtle2_pose_ = pose;

	// Calculate Translation vector of turtle 2 respect to turtle 1
	float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
	float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;

	float theta_rad = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
	float theta_deg = theta_rad * 180 / 3.14;

	RCLCPP_INFO_STREAM(this->get_logger(), 
		"====================================================");
	RCLCPP_INFO_STREAM(this->get_logger(), 
		"Translation vector between turtle1->turtle2\n" 
		<< "\tTx: " << Tx << "\tTy: " << Ty);
	RCLCPP_INFO_STREAM(this->get_logger(), 
		"Rotation angle between turtle1->turtle2\n" 
		<< "\tTheta(radian): " << theta_rad
		<< "\tTheta(degree): " << theta_deg);
	RCLCPP_INFO_STREAM(this->get_logger(),
		"Rotation matrix turtle1->turtle2\n"
		<< "|" << std::cos(theta_rad) << "\t\t" << -std::sin(theta_rad) << "|\n"
		<< "|" << std::sin(theta_rad) << "\t\t" << std::cos(theta_rad) << "|"
	);
	RCLCPP_INFO_STREAM(this->get_logger(), 
		"====================================================");
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SimpleTurtleSimKinematics>("simple_turtlesim_kinematics");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}