cmake_minimum_required(VERSION 3.8)
project(WT_MPC)


set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#gdb调试
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

set(MUJOCO_PATH "/home/loaf/mujoco")
find_package(CURL REQUIRED)

include_directories(${MUJOCO_PATH}/include ${PROJECT_SOURCE_DIR}/include 
/usr/include/glib-2.0
/usr/lib/x86_64-linux-gnu/glib-2.0/include
)

link_directories(${MUJOCO_PATH}/build/bin)
set(MUJOCO_LIB ${MUJOCO_PATH}/build/lib/libmujoco.so)

add_executable(basic src/basic.cc
src/FSM/ControlComponent.cpp src/FSM/Free_State.cpp src/FSM/FSM.cpp src/FSM/FSMState.cpp src/FSM/Passive_State.cpp
src/FSM/Stand_State.cpp src/FSM/Trotting_State.cpp src/FSM/UserCmd.cpp src/FSM/Sit_Down_State.cpp

src/Gait/FeetEndCal.cpp src/Gait/GaitGenerator.cpp src/Gait/WaveGenerator.cpp

src/low/LowCmd.cpp src/low/LowState.cpp

src/math/Kenimatics_normal_solution.cpp src/math/Reversal_solution.cpp src/math/Robot.cpp src/math/Mujoco_box.cpp
src/math/data_analyze.cpp

src/sensor/Imu.cpp

src/WBC/BalanceCtrl.cpp src/WBC/Estimator.cpp src/WBC/mujocoEstimator.cpp

src/start.cpp
)

target_link_libraries(basic ${MUJOCO_LIB} glut GL GLU glfw
quadprog
pthread
CURL::libcurl
/usr/lib/x86_64-linux-gnu/liblcm.so
/usr/lib/x86_64-linux-gnu/libgthread-2.0.so
/usr/lib/x86_64-linux-gnu/libglib-2.0.so
)
