cmake_minimum_required(VERSION 3.8)
project(mit_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(motor_control_msgs REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mit_controllers.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle pluginlib controller_interface motor_control_msgs )

pluginlib_export_plugin_description_file(controller_interface mit_controller_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()


