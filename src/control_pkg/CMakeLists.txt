cmake_minimum_required(VERSION 3.8)
project(control_pkg)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(LOWSTATE_DEBUG OFF)  # lowstate 数据调试
set(LOWCMD_DEBUG OFF)  # lowcmd 数据调试

if(LOWSTATE_DEBUG)
    add_definitions(-DLOWSTATE_DEBUG)
endif()
if(LOWCMD_DEBUG)
    add_definitions(-DLOWCMD_DEBUG)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motor_control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(start src/start.cpp 
src/common/Estimator.cpp src/common/Imu.cpp src/common/LowPassFilter.cpp src/common/Robot.cpp  
src/FSM/ControlComponent.cpp src/FSM/Free_Stand_State.cpp src/FSM/Free_State.cpp src/FSM/FSM.cpp src/FSM/FSMState.cpp
src/FSM/Passive_State.cpp src/FSM/Stand_State.cpp src/FSM/Trotting_State.cpp src/FSM/UserCmd.cpp src/FSM/Sit_Down_State.cpp
src/Gait/FeetEndCal.cpp  src/Gait/WaveGenerator.cpp  src/Gait/GaitGenerator.cpp
src/low/LowCmd.cpp src/low/LowState.cpp
src/math/Kenimatics_normal_solution.cpp src/math/Reversal_solution.cpp
src/damiao_SDK/damiao.cpp
src/IOPort/IORos.cpp
src/common/BalanceCtrl.cpp
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include/CSerialPort
    /usr/include/glib-2.0
    /usr/lib/x86_64-linux-gnu/glib-2.0/include
    /usr/include/qpOASES
)

ament_target_dependencies(start
  rclcpp
  trajectory_msgs
  control_msgs
  sensor_msgs
  motor_control_msgs
  
)

target_link_libraries(
start
quadprog
qpOASES
pthread
/usr/local/lib/libcserialport.so
/usr/lib/x86_64-linux-gnu/liblcm.so
/usr/lib/x86_64-linux-gnu/libgthread-2.0.so
/usr/lib/x86_64-linux-gnu/libglib-2.0.so
)

install(DIRECTORY launch controller_config ros2_control_config rviz_config
DESTINATION share/${PROJECT_NAME})

install(TARGETS start
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
