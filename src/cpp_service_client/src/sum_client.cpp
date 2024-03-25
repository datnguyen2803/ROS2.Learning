#include "rclcpp/rclcpp.hpp"
#include "self_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	if (argc != 3)
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: sum_client X Y");
		return 1;
	}

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sum_client");
	rclcpp::Client<self_interfaces::srv::AddTwoInts>::SharedPtr client = node->create_client<self_interfaces::srv::AddTwoInts>("add_two_ints");

	auto request = std::make_shared<self_interfaces::srv::AddTwoInts::Request>();
	request->a = atoll(argv[1]);
	request->b = atoll(argv[2]);

	while (!client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "client interrupted while waiting for service to appear.");
			return 1;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for service to appear...");
	}

	auto result = client->async_send_request(request);
	if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
	}
	else
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service call failed.");
	}

	rclcpp::shutdown();
	return 0;
}