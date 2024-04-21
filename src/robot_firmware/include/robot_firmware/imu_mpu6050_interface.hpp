#ifndef __IMU_MPU6050_INTERFACE_HPP__
#define __IMU_MPU6050_INTERFACE_HPP__

#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "lib_mpu6050.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace robot_firmware_interfaces
{

	class MPU6050Interface : public hardware_interface::SensorInterface
	{
	public:
		MPU6050Interface();

		CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

		std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

		CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

		CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

		return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

	private:
		MPU6050 device_;

		Quaternion orientation_;
		double angular_vel_x_;
		double angular_vel_y_;
		double angular_vel_z_;
		double linear_accel_x_;
		double linear_accel_y_;
		double linear_accel_z_;

		Quaternion quat_;
		float euler_angles_[3];
		float gyro_values_[3];
		float accel_values_[3];
	};

} // namespace robot_firmware_interfaces

#endif
