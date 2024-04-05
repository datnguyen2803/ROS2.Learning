#include "robot_firmware/wheel_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <unistd.h>
#include <pluginlib/class_list_macros.hpp>

#define logger (rclcpp::get_logger("WheelInterface"))

namespace robot_firmware_interfaces
{

WheelInterface::WheelInterface()
{
	RCLCPP_INFO(logger, "mikey WheelInterface::WheelInterface()");
}

WheelInterface::~WheelInterface()
{
	RCLCPP_INFO(logger, "mikey WheelInterface::~WheelInterface()");
}

CallbackReturn WheelInterface::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger, "mikey WheelInterface::on_activate()");

	// Reset commands and states	
	velocity_commands_ = { 0.0, 0.0 };
	position_states_ = { 0.0, 0.0 };
	velocity_states_ = { 1.0, 5.0 };

	return CallbackReturn::SUCCESS;
}

CallbackReturn WheelInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger, "mikey WheelInterface::on_deactivate()");
	return CallbackReturn::SUCCESS;
}

CallbackReturn WheelInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
	RCLCPP_INFO(logger, "Mikey WheelInterface::on_init()");
	CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
	if(result != CallbackReturn::SUCCESS)
	{
		return result;
	}

	velocity_commands_.reserve(info_.joints.size());
	position_states_.reserve(info_.joints.size());
	velocity_states_.reserve(info_.joints.size());

	return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WheelInterface::export_state_interfaces()
{

	RCLCPP_INFO(logger, "Mikey WheelInterface::export_state_interfaces()");

	std::vector<hardware_interface::StateInterface> state_interfaces;

	for(size_t i = 0; i < info_.joints.size(); i++)
	{
		state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
		state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
	}

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WheelInterface::export_command_interfaces()
{

	RCLCPP_INFO(logger, "Mikey WheelInterface::export_command_interfaces()");

	std::vector<hardware_interface::CommandInterface> command_interfaces;

	for(size_t i = 0; i < info_.joints.size(); i++)
	{
		command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
	}

	return command_interfaces;

}

/**
 * Read from hardware, such as encoder
*/
hardware_interface::return_type WheelInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
	// RCLCPP_INFO(logger, "WheelInterface::read()");
	return hardware_interface::return_type::OK;
}

/**
 * Write to hardware, such as motor
*/
hardware_interface::return_type WheelInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
	// RCLCPP_INFO(logger, "WheelInterface::write()");
	// usleep(10);
	RCLCPP_INFO(logger, "Mikey write velocity_commands_ = %0.5f - %0.5f", velocity_commands_.at(0), velocity_commands_.at(1));
	return hardware_interface::return_type::OK;
}

} // namespace robot_firmware_interfaces

PLUGINLIB_EXPORT_CLASS(robot_firmware_interfaces::WheelInterface, hardware_interface::SystemInterface)