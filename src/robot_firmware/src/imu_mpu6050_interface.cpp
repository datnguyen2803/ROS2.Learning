#include "robot_firmware/imu_mpu6050_interface.hpp"
#include <pluginlib/class_list_macros.hpp>


#define logger (rclcpp::get_logger("MPU6050Interface"))

namespace robot_firmware_interfaces
{

// // Initialize MPU6050 device
// MPU6050 device(0x68); 

MPU6050Interface::MPU6050Interface()
	: device_(0x68)
{
	RCLCPP_INFO(logger, "mikey MPU6050Interface::MPU6050Interface()");
}

CallbackReturn MPU6050Interface::on_init(const hardware_interface::HardwareInfo & info)
{
	if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
	{
		return CallbackReturn::ERROR;
	}

	RCLCPP_INFO(logger, "Initializing...");

	RCLCPP_INFO(logger, "Finished initialization");
	
	return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MPU6050Interface::export_state_interfaces()
{
	// Set up the MPU6050 state interfaces
	
	std::vector<hardware_interface::StateInterface> state_interfaces;

	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &orientation_.x));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &orientation_.y));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &orientation_.z));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &orientation_.w));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &angular_vel_x_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &angular_vel_y_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &angular_vel_z_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &linear_accel_x_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &linear_accel_y_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &linear_accel_z_));

	return state_interfaces;
}

CallbackReturn MPU6050Interface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_INFO(logger, "Starting controller ...");

	return CallbackReturn::SUCCESS;
}

CallbackReturn MPU6050Interface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_INFO(logger, "Stopping Controller...");
	
	return CallbackReturn::SUCCESS;
}

return_type MPU6050Interface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	// Obtain current euler angles
	device_.getAngle(0, &euler_angles_[0]);
	device_.getAngle(1, &euler_angles_[1]);
	device_.getAngle(2, &euler_angles_[2]);

	// Obtain current quaternion from euler angles
	quat_ = device_.getQuat(&euler_angles_[0], &euler_angles_[1], &euler_angles_[2]);

	// Obtain current gyroscope and accelerometer values
	device_.getGyro(&gyro_values_[0], &gyro_values_[1], &gyro_values_[2]);
	device_.getAccel(&accel_values_[0], &accel_values_[1], &accel_values_[2]); 

	// Assign values to the state interfaces
	// Orientation, angular velocity and linear acceleration conform the East North Up (ENU) coordinate frame
	// convention (https://www.ros.org/reps/rep-0103.html) required by the robot_localization package
	orientation_.x = quat_.y;
	orientation_.y = quat_.x;
	orientation_.z = quat_.z;
	orientation_.w = quat_.w;
	angular_vel_x_ = (double)gyro_values_[1];
	angular_vel_y_ = (double)gyro_values_[0];
	angular_vel_z_ = (double)gyro_values_[2];
	linear_accel_x_ = (double)accel_values_[1];
	linear_accel_y_ = (double)accel_values_[0];
	linear_accel_z_ = (double)accel_values_[2];

	return return_type::OK;
}

} // namespace lidarbot_bringup

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	robot_firmware_interfaces::MPU6050Interface,
	hardware_interface::SensorInterface)
