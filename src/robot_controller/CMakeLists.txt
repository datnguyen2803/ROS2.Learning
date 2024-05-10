cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
	controller_interface
	hardware_interface
	generate_parameter_library
	std_msgs
	pluginlib
	rclcpp
	rclcpp_lifecycle
	rcpputils
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(single_motor_controller_parameters
	src/single_motor_controller_parameter.yaml
)

add_library(${PROJECT_NAME} SHARED
	src/single_motor_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} PUBLIC single_motor_controller_parameters)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface single_motor_plugin.xml)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
	DIRECTORY include/
	DESTINATION include/${PROJECT_NAME}
)
install(
	TARGETS ${PROJECT_NAME} single_motor_controller_parameters
	EXPORT export_single_motor_controller
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
ament_export_targets(export_single_motor_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

include_directories(include)

add_compile_options(-g)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
