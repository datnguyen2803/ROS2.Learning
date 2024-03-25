#include <functional>
#include <memory>
#include <thread>
#include <future>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "self_interfaces/action/fibonacci.hpp"

namespace cpp_action_server
{
class FibonacciClient : public rclcpp::Node
{
public:
	using Fibonacci = self_interfaces::action::Fibonacci;
	using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

	explicit FibonacciClient(const rclcpp::NodeOptions & options)
		: Node("fibonacci_client", options)
	{
		this->action_client_ = rclcpp_action::create_client<Fibonacci>(
			this,
			"fibonacci");
		this->timer_ = this->create_wall_timer(
			std::chrono::milliseconds(500),
			std::bind(&FibonacciClient::send_goal, this));
	}

	void send_goal()
	{
		using namespace std::placeholders;
		this->timer_->cancel();
		if(!this->action_client_->wait_for_action_server())
		{
			RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
			rclcpp::shutdown();
		}

		auto goal_msg = Fibonacci::Goal();
		goal_msg.order = 10;

		RCLCPP_INFO(this->get_logger(), "Sending goal");

		auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
		send_goal_options.goal_response_callback = std::bind(&FibonacciClient::goal_response_callback, this, _1);
		send_goal_options.feedback_callback = std::bind(&FibonacciClient::feedback_callback, this, _1, _2);
		send_goal_options.result_callback = std::bind(&FibonacciClient::result_callback, this, _1);
		this->action_client_->async_send_goal(goal_msg, send_goal_options);
	}

private:
	rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
	rclcpp::TimerBase::SharedPtr timer_;

	void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
	{
		if(!goal_handle)
		{
			RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
		}
	}

	void feedback_callback(
		rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
		const std::shared_ptr<const Fibonacci::Feedback> feedback)
	{
		std::stringstream ss;
		for(auto number : feedback->partial_sequence)
		{
			ss << number << ", ";
		}
		RCLCPP_INFO(this->get_logger(), "Next number in sequence: %s", ss.str().c_str());
	}

	void result_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult & result)
	{
		switch(result.code)
		{
			case rclcpp_action::ResultCode::SUCCEEDED:
			{
				std::stringstream ss;
				for(auto number : result.result->sequence)
				{
					ss << number << ", ";
				}
				RCLCPP_INFO(this->get_logger(), "Result received: %s", ss.str().c_str());
				break;
			}
			case rclcpp_action::ResultCode::ABORTED:
				RCLCPP_INFO(this->get_logger(), "Goal was aborted");
				return;
			case rclcpp_action::ResultCode::CANCELED:
				RCLCPP_INFO(this->get_logger(), "Goal was canceled");
				return;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				break;
		}
		// this->timer_ = this->create_wall_timer(
		// 	std::chrono::milliseconds(500),
		// 	std::bind(&FibonacciClient::send_goal, this));

		rclcpp::shutdown();
	}
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(cpp_action_server::FibonacciClient)