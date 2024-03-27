#include "cpp_bumperbot_examples/simple_tf_kinematics.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

SimpleTFKinematics::SimpleTFKinematics(const std::string &node_name)
	: Node(node_name)
	, x_increment_(0.05)
	, last_x_(0.0)
	, rotations_counter_(0)
{
	static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

	dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	static_transform_stamped_.header.stamp = get_clock()->now();
	static_transform_stamped_.header.frame_id = "bumperbot_base";
	static_transform_stamped_.child_frame_id = "bumperbot_top";
	static_transform_stamped_.transform.translation.x = 0.0;
	static_transform_stamped_.transform.translation.y = 0.0;
	static_transform_stamped_.transform.translation.z = 0.3;
	static_transform_stamped_.transform.rotation.x = 0.0;
	static_transform_stamped_.transform.rotation.y = 0.0;
	static_transform_stamped_.transform.rotation.z = 0.0;
	static_transform_stamped_.transform.rotation.w = 1.0;

	static_tf_broadcaster_->sendTransform(static_transform_stamped_);

	RCLCPP_INFO_STREAM(get_logger(), "Published static transform from " 
		<< static_transform_stamped_.header.frame_id << " to "
		<< static_transform_stamped_.child_frame_id);

	timer_ = create_wall_timer(0.1s, std::bind(&SimpleTFKinematics::timerCallback, this));

	get_transform_service_ = create_service<self_interfaces::srv::GetTransform>("get_transform_service",
		std::bind(&SimpleTFKinematics::getTransformCallback, this, _1, _2));

	last_orientation_.setRPY(0.0, 0.0, 0.0);
	orientation_increment_.setRPY(0.0, 0.0, 0.05);
}

void SimpleTFKinematics::timerCallback()
{
	dynamic_transform_stamped_.header.stamp = get_clock()->now();
	dynamic_transform_stamped_.header.frame_id = "odom";
	dynamic_transform_stamped_.child_frame_id = "bumperbot_base";
	dynamic_transform_stamped_.transform.translation.x = last_x_;
	dynamic_transform_stamped_.transform.translation.y = 0.0;
	dynamic_transform_stamped_.transform.translation.z = 0.0;

	// dynamic_transform_stamped_.transform.rotation.x = 0.0;
	// dynamic_transform_stamped_.transform.rotation.y = 0.0;
	// dynamic_transform_stamped_.transform.rotation.z = 0.0;
	// dynamic_transform_stamped_.transform.rotation.w = 1.0;

	tf2::Quaternion q;
	q = last_orientation_ * orientation_increment_;
	q.normalize();

	dynamic_transform_stamped_.transform.rotation.x = q.x();
	dynamic_transform_stamped_.transform.rotation.y = q.y();
	dynamic_transform_stamped_.transform.rotation.z = q.z();
	dynamic_transform_stamped_.transform.rotation.w = q.w();

	dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);

	RCLCPP_INFO_STREAM(get_logger(), "Published dynamic transform from " 
		<< dynamic_transform_stamped_.header.frame_id << " to "
		<< dynamic_transform_stamped_.child_frame_id << " with x = " << last_x_);

	last_x_ += x_increment_;
	rotations_counter_++;
	last_orientation_ = q;

	if(rotations_counter_ >= 100)
	{
		orientation_increment_ = orientation_increment_.inverse();
		rotations_counter_ = 0;
	}
}

bool SimpleTFKinematics::getTransformCallback(const std::shared_ptr<self_interfaces::srv::GetTransform::Request> request,
											  const std::shared_ptr<self_interfaces::srv::GetTransform::Response> response)
{
	RCLCPP_INFO_STREAM(get_logger(), "Requested transform between"
		<< request->frame_id << " and " << request->child_frame_id);

	geometry_msgs::msg::TransformStamped requested_transform;
	try
	{
		requested_transform = tf_buffer_->lookupTransform(request->frame_id, request->child_frame_id, tf2::TimePointZero);
	}
	catch(tf2::TransformException &e)
	{
		RCLCPP_ERROR_STREAM(get_logger(), "Error " << e.what());
		response->success = false;
		return false;
	}

	response->transform = requested_transform;
	response->success = true;
	return true;
	
	
}


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SimpleTFKinematics>("simple_tf_kinematics");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}