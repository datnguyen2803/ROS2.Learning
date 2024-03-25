#include "rclcpp/rclcpp.hpp"
#include "self_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add_two_ints(const std::shared_ptr<self_interfaces::srv::AddTwoInts::Request> request,
				  std::shared_ptr<self_interfaces::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
			  request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sum_server");
  rclcpp::Service<self_interfaces::srv::AddTwoInts>::SharedPtr server = node->create_service<self_interfaces::srv::AddTwoInts>("add_two_ints", &add_two_ints);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
