#include "robot_firmware/mop_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <unistd.h>

#include "robot_firmware/motor_driver.h"
#include "robot_firmware/motor_encoder.h"

#define logger (rclcpp::get_logger("MopInterface"))
#define MOP_JOINT_NAME	"mop_joint"

namespace robot_firmware_interfaces
{

MopInterface::MopInterface()
{
	RCLCPP_INFO(logger, "mikey MopInterface::MopInterface()");
}

MopInterface::~MopInterface()
{
	RCLCPP_INFO(logger, "mikey MopInterface::~MopInterface()");
	stop_mop();
}

CallbackReturn MopInterface::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger, "ntdat on_configure");

	// Init motor driver
	Motor_Init();

	// Init Wiring Pi
	wiringPiSetupGpio();

	// Setup GPIO pins mode
	pinMode(MOP_MOTOR_PIN_A, PWM_OUTPUT);
	pinMode(MOP_MOTOR_PIN_B, OUTPUT);

	return CallbackReturn::SUCCESS;
}

CallbackReturn MopInterface::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger, "ntdat on_activate");
	position_state_ = 0.0;
	velocity_command_ = 0.0;
	return CallbackReturn::SUCCESS;
}

CallbackReturn MopInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger, "ntdat on_deactivate");
	stop_mop();
	return CallbackReturn::SUCCESS;
}

CallbackReturn MopInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
	RCLCPP_INFO(logger, "ntdat on_init");
	CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
	if (result != CallbackReturn::SUCCESS)
	{
		return result;
	}

	return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MopInterface::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;
	state_interfaces.emplace_back(hardware_interface::StateInterface(MOP_JOINT_NAME, hardware_interface::HW_IF_POSITION, &position_state_));
	RCLCPP_INFO(logger, "ntdat check export_state_interfaces = %s", state_interfaces[0].get_name().c_str());
	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MopInterface::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;
	command_interfaces.emplace_back(hardware_interface::CommandInterface(MOP_JOINT_NAME, hardware_interface::HW_IF_VELOCITY, &velocity_command_));
	RCLCPP_INFO(logger, "ntdat check export_command_interfaces = %s", command_interfaces[0].get_name().c_str());
	return command_interfaces;
}

hardware_interface::return_type MopInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
	// Read the state from the hardware
	position_state_ = 0.0;

	// RCLCPP_INFO(logger, "Reading MopInterface velocity_state: %f", position_state_);

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type MopInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
	// Send the command to the hardware
	// velocity_command_ = 0.0;

	RCLCPP_INFO(logger, "Mikey MopInterface velocity_command_: %f", velocity_command_);
	int temp_velocity = ceil(velocity_command_ * 1024.0 / 10.0);
	run_mop(temp_velocity);
	return hardware_interface::return_type::OK;
}

void MopInterface::run_mop(int velocity)
{
	if(velocity != 0)
	{
		// digitalWrite(MOP_MOTOR_PIN_A, HIGH);
		pwmWrite(MOP_MOTOR_PIN_A, velocity);
		digitalWrite(MOP_MOTOR_PIN_B, LOW);
	}
	else
	{
		stop_mop();
	}
}

void MopInterface::stop_mop()
{
	digitalWrite(MOP_MOTOR_PIN_A, LOW);
	digitalWrite(MOP_MOTOR_PIN_B, LOW);

}

} // namespace robot_firmware_interfaces


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_firmware_interfaces::MopInterface, hardware_interface::SystemInterface)