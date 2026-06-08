cmake_minimum_required(VERSION 3.8)
project(imu_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu src/imu.cpp )

include_directories(${PROJECT_SOURCE_DIR}/include
    /usr/local/include/CSerialPort
)

ament_target_dependencies(imu
  rclcpp
  sensor_msgs
)

target_link_libraries(
imu
/usr/local/lib/libcserialport.so

)
install(TARGETS imu
DESTINATION lib/${PROJECT_NAME}
)
ament_package()
