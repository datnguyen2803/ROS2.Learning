#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TextPublisher : public rclcpp::Node
{
public:
	TextPublisher() : Node("text_publisher")
	{
		publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
		timer_ = this->create_wall_timer(
			1s, std::bind(&TextPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto message = std_msgs::msg::String();
		message.data = "Hello, world!";
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		publisher_->publish(message);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TextPublisher>());
	rclcpp::shutdown();
	return 0;
}

