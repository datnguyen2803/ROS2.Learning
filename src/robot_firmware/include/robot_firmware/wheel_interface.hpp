#ifndef __WHEEL_INTERFACE_HPP__
#define __WHEEL_INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <vector>
#include <string>

#include "robot_firmware/wheel.hpp"



namespace robot_firmware_interfaces
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class WheelInterface : public hardware_interface::SystemInterface
{
private:
	struct Config
	{
		int enc_ticks_per_rev = 1084;
		double loop_rate = 30.0;
	};


public:
	WheelInterface();
	virtual ~WheelInterface();

	// Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
	virtual CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
	virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
	virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

	// Implementing must-have methods hardware_interface::SystemInterface
	virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
	virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
	virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
	virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
	virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
	std::vector<double> velocity_commands_;
	std::vector<double> position_states_;
	std::vector<double> velocity_states_;

	Config config_;
	Wheel left_wheel_;
	Wheel right_wheel_;

}; // class WheelInterface

} // namespace robot_firmware_interfaces

#endif