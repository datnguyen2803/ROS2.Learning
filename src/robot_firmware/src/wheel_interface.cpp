#include "robot_firmware/wheel_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <unistd.h>
#include <pluginlib/class_list_macros.hpp>

#include "robot_firmware/motor_driver.h"
#include "robot_firmware/motor_encoder.h"

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


CallbackReturn WheelInterface::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger, "mikey WheelInterface::on_configure()");

	// Init motor driver
	Motor_Init();

	// Init Wiring Pi
	wiringPiSetupGpio();

	// Setup GPIO pins mode
	pinMode(LEFT_WHL_ENC_INT, INPUT);
	pinMode(RIGHT_WHL_ENC_INT, INPUT);
	pinMode(LEFT_WHL_ENC_DIR, INPUT);
	pinMode(RIGHT_WHL_ENC_DIR, INPUT);

	// Setup pull up resistors on encoder interrupt pins
	pullUpDnControl(LEFT_WHL_ENC_INT, PUD_UP);
	pullUpDnControl(RIGHT_WHL_ENC_INT, PUD_UP);

	// Initialize encoder interrupts for falling signal states
	wiringPiISR(LEFT_WHL_ENC_INT, INT_EDGE_FALLING, left_wheel_pulse);
	wiringPiISR(RIGHT_WHL_ENC_INT, INT_EDGE_FALLING, right_wheel_pulse);

	RCLCPP_INFO(logger, "Successfully configured motors and encoders!");

	return CallbackReturn::SUCCESS;
}

CallbackReturn WheelInterface::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger, "mikey WheelInterface::on_activate()");

	// Reset commands and states	
	velocity_commands_ = { 0.0, 0.0 };
	position_states_ = { 0.0, 0.0 };
	velocity_states_ = { 0.0, 0.0 };

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

	left_wheel_.setup(info_.joints[0].name, config_.enc_ticks_per_rev);
	right_wheel_.setup(info_.joints[1].name, config_.enc_ticks_per_rev);

	velocity_commands_.reserve(info_.joints.size());
	position_states_.reserve(info_.joints.size());
	velocity_states_.reserve(info_.joints.size());

	return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WheelInterface::export_state_interfaces()
{

	RCLCPP_INFO(logger, "Mikey WheelInterface::export_state_interfaces()");

	std::vector<hardware_interface::StateInterface> state_interfaces;

	// for(size_t i = 0; i < info_.joints.size(); i++)
	// {
	// 	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
	// 	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
	// }

	state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.position));
	state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity));

	state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.position));
	state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity));

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WheelInterface::export_command_interfaces()
{

	RCLCPP_INFO(logger, "Mikey WheelInterface::export_command_interfaces()");

	std::vector<hardware_interface::CommandInterface> command_interfaces;

	// for(size_t i = 0; i < info_.joints.size(); i++)
	// {
	// 	command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
	// }

	command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.command));
	command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.command));

	return command_interfaces;

}

/**
 * Read from hardware, such as encoder
*/
hardware_interface::return_type WheelInterface::read(const rclcpp::Time &/*time*/, const rclcpp::Duration &duration)
{
	// RCLCPP_INFO(logger, "WheelInterface::read()");

	// Obtain elapsed time
	double delta_seconds = duration.seconds();

	// Obtain encoder values
	read_encoder_values(&left_wheel_.encoder_ticks, &right_wheel_.encoder_ticks);

	// Calculate wheel positions and velocities
	double previous_position = left_wheel_.position;
	left_wheel_.position = left_wheel_.calculate_encoder_angle();
	left_wheel_.velocity = (left_wheel_.position - previous_position) / delta_seconds;

	previous_position = right_wheel_.position;
	right_wheel_.position = right_wheel_.calculate_encoder_angle();
	right_wheel_.velocity = (right_wheel_.position - previous_position) / delta_seconds;

	return hardware_interface::return_type::OK;
}

/**
 * Write to hardware, such as motor
*/
hardware_interface::return_type WheelInterface::write(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*duration*/)
{
	// RCLCPP_INFO(logger, "WheelInterface::write()");
	// RCLCPP_INFO(logger, "Mikey write velocity_commands_ = %0.5f - %0.5f", velocity_commands_.at(0), velocity_commands_.at(1));

	double left_motor_counts_per_loop = left_wheel_.command / left_wheel_.rads_per_tick / config_.loop_rate;
	double right_motor_counts_per_loop = right_wheel_.command / right_wheel_.rads_per_tick / config_.loop_rate;

	// RCLCPP_INFO(logger, "ntdat left_wheel_.command = %f, right_wheel_.command = %f", left_wheel_.command, right_wheel_.command);

	// Send commands to motor driver
	set_motor_speeds(left_motor_counts_per_loop, right_motor_counts_per_loop);
	
	return hardware_interface::return_type::OK;
}

} // namespace robot_firmware_interfaces

PLUGINLIB_EXPORT_CLASS(robot_firmware_interfaces::WheelInterface, hardware_interface::SystemInterface)