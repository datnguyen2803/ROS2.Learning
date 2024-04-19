cmake_minimum_required(VERSION 3.8)
project(robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)

# COMPILE
add_library(robot_firmware SHARED 
	src/wheel_interface.cpp
	src/DEV_Config.c
	src/motor_driver.c
	src/motor_encoder.c
	src/PCA9685.c
	src/wheel.cpp
)

target_include_directories(robot_firmware PRIVATE include)

ament_target_dependencies(
	robot_firmware
	rclcpp
	hardware_interface
	rclcpp_lifecycle
	pluginlib
	std_srvs
)

# Link WiringPi library to lidarbot_hardware target
target_link_libraries(robot_firmware wiringPi)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface robot_firmware_interfaces.xml)


# INSTALL
install(
	DIRECTORY include/
	DESTINATION include/${PROJECT_NAME}
)

install(
	TARGETS robot_firmware
	EXPORT export_robot_firmware
	DESTINATION share/${PROJECT_NAME}
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

# EXPORTS
ament_export_libraries(robot_firmware)
ament_export_targets(export_robot_firmware HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
