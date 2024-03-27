#ifndef __SIMPLE_TF_KINEMATICS_HPP__
#define __SIMPLE_TF_KINEMATICS_HPP__

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "self_interfaces/srv/get_transform.hpp"

#include <memory>


class SimpleTFKinematics : public rclcpp::Node
{
public:
	SimpleTFKinematics(const std::string &node_name);


	

private:
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
	geometry_msgs::msg::TransformStamped static_transform_stamped_;
	geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

	rclcpp::Service<self_interfaces::srv::GetTransform>::SharedPtr get_transform_service_;

	rclcpp::TimerBase::SharedPtr timer_;
	double x_increment_;
	double last_x_;
	int rotations_counter_;
	tf2::Quaternion last_orientation_;
	tf2::Quaternion orientation_increment_;

	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

	void timerCallback();

	bool getTransformCallback(const std::shared_ptr<self_interfaces::srv::GetTransform::Request> request,
							  const std::shared_ptr<self_interfaces::srv::GetTransform::Response> response);


};


#endif // __SIMPLE_TF_KINEMATICS_HPP__