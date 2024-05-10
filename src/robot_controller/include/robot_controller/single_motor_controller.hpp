#ifndef __SINGLE_MOTOR_CONTROLLER_HPP__
#define __SINGLE_MOTOR_CONTROLLER_HPP__


#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"


namespace robot_controller
{


class SingleMotorController : public controller_interface::ControllerInterface
{
	using Twist = geometry_msgs::msg::Twist;

public:
	SingleMotorController();
	virtual ~SingleMotorController();

	virtual controller_interface::CallbackReturn on_init() override;
	virtual controller_interface::InterfaceConfiguration command_interface_configuration() const override;
	virtual controller_interface::InterfaceConfiguration state_interface_configuration() const override;
	virtual controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

	virtual controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
	virtual controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
	virtual controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
	virtual controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
	virtual controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
	virtual controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

protected:
	// Parameters from ROS for diff_drive_controller
	// std::shared_ptr<single_motor_controller::ParamListener> param_listener_;
	// single_motor_controller::Params params_;

private:
	std::string motor_name_;
	std::string joint_name_;

	std::queue<bool> previous_vel_commands_;
	// std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> in_command_subscriber_ = nullptr;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr in_command_subscriber_ = nullptr;

	double vel_out_; // in rad/s
	std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> out_publisher_ = nullptr;

	// std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
	std::vector<hardware_interface::LoanedCommandInterface> test_interfaces;

	bool is_halted_;
	bool reset();
	void halt();


};


}

#endif