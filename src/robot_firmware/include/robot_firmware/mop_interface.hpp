#ifndef __MOP_INTERFACE_HPP__
#define __MOP_INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <vector>
#include <string>

#define MOP_MOTOR_PIN_A 13
#define MOP_MOTOR_PIN_B 6

namespace robot_firmware_interfaces
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MopInterface : public hardware_interface::SystemInterface
{

public:
	MopInterface();
	virtual ~MopInterface();

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
	double velocity_command_;
	double position_state_;

	void run_mop(int velocity);
	void stop_mop();

};

} // namespace robot_firmware_interfaces

#endif // __MOP_INTERFACE_HPP__
