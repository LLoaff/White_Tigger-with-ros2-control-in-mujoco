cmake_minimum_required(VERSION 3.8)
project(unitree_hardware_interface)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)


add_library(
  ${PROJECT_NAME}
  SHARED
  src/unitree_hardware_interface.cpp
)
target_link_libraries(${PROJECT_NAME}

)

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle pluginlib hardware_interface )

pluginlib_export_plugin_description_file(hardware_interface unitree_hardware_interface_plugin.xml)

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
