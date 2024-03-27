cmake_minimum_required(VERSION 3.8)
project(cpp_bumperbot_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(self_interfaces REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(simple_turtle_kinematics src/simple_turtlesim_kinematics.cpp)
ament_target_dependencies(simple_turtle_kinematics rclcpp turtlesim)

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics rclcpp tf2_ros geometry_msgs self_interfaces tf2)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS simple_turtle_kinematics simple_tf_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
