#ifndef __SIMPLE_DRAWER_HPP__
#define __SIMPLE_DRAWER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>


class SimpleDrawer : public rclcpp::Node
{
public:
	SimpleDrawer(const std::string& node_name);

private:
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

};

#endif // __SIMPLE_DRAWER_HPP__