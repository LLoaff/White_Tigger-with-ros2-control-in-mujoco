cmake_minimum_required(VERSION 3.8)
project(WT_MPC)


set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#gdb调试
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# 用户参数：
set(USE_SIM ON)
set(USE_GO1_MODEL OFF)
set(MUJOCO_PATH "/home/loaf/mujoco")
set(CSERIALPORT_INCLUDE_DIR "/usr/local/include/CSerialPort")
set(CSERIALPORT_LIBRARY "/usr/local/lib/libcserialport.so")

if(USE_REAL_ROBOT)
    add_definitions(-DUSE_REAL_ROBOT)
endif()

if(USE_GO1_MODEL)
    add_definitions(-DUSE_GO1_MODEL)
endif()
if(USE_SIM)
    add_definitions(-DUSE_SIM)
endif()

find_package(CURL REQUIRED)

add_subdirectory(libs/osqp)


include_directories(${MUJOCO_PATH}/include ${PROJECT_SOURCE_DIR}/include 
/usr/include/glib-2.0
/usr/lib/x86_64-linux-gnu/glib-2.0/include
libs/osqp/include
)

link_directories(${MUJOCO_PATH}/build/bin)
set(MUJOCO_LIB ${MUJOCO_PATH}/build/lib/libmujoco.so)

add_executable(basic src/basic.cc
src/FSM/ControlComponent.cpp src/FSM/Free_State.cpp src/FSM/FSM.cpp src/FSM/FSMState.cpp src/FSM/Passive_State.cpp
src/FSM/Stand_State.cpp src/FSM/Trotting_State.cpp src/FSM/UserCmd.cpp src/FSM/Sit_Down_State.cpp src/FSM/Trotting_State_mpc.cpp
src/FSM/State_BalanceTest.cpp src/FSM/Free_Stand_State.cpp

src/Gait/FeetEndCal.cpp src/Gait/GaitGenerator.cpp src/Gait/WaveGenerator.cpp

src/low/LowCmd.cpp src/low/LowState.cpp

src/math/Kenimatics_normal_solution.cpp src/math/Reversal_solution.cpp src/math/Robot.cpp src/math/Mujoco_box.cpp
src/math/data_analyze.cpp

src/sensor/Imu.cpp

src/WBC/BalanceCtrl.cpp src/WBC/Estimator.cpp src/WBC/mujocoEstimator.cpp

src/MPC/ConvexMPC.cpp

src/start.cpp
)

target_link_libraries(basic ${MUJOCO_LIB} glut GL GLU glfw
quadprog
osqp
pthread
CURL::libcurl
/usr/lib/x86_64-linux-gnu/liblcm.so
/usr/lib/x86_64-linux-gnu/libgthread-2.0.so
/usr/lib/x86_64-linux-gnu/libglib-2.0.so
cserialport
)

####################################################################################

# cmake_minimum_required(VERSION 3.8)

# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR aarch64)
# set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# project(WT_MPC)

# # 明确指定lgpio库的路径
# set(LGPIO_INCLUDE_DIR /usr/local/aarch64-linux-gnu/include)
# set(LGPIO_LIBRARY /usr/local/aarch64-linux-gnu/lib/liblgpio.so)

# # ========== C/C++标准配置 ==========
# set(CMAKE_C_STANDARD 11)
# set(CMAKE_C_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用非标准扩展

# # ========== 项目基本配置 ==========
# set(TARGET_NAME "basic")
# set(BOARD_IP "10.33.236.110")
# set(BOARD_USER "shu")
# # 自定义宏
# set(USE_SIM OFF)
# set(USE_GO1_MODEL OFF)
# if(USE_GO1_MODEL)
#     add_definitions(-DUSE_GO1_MODEL)
# endif()
# if(USE_SIM)
#     add_definitions(-DUSE_SIM)
# endif()

# find_package(osqp CONFIG REQUIRED
#     PATHS /usr/local/aarch64-linux-gnu/lib/cmake/osqp
#     NO_DEFAULT_PATH
# )



# # ========== 源文件列表 ==========
# set(SRC_FILES
# src/basic.cc
# src/damiao_SDK/damiao.cpp
# src/FSM/ControlComponent.cpp src/FSM/Free_State.cpp src/FSM/FSM.cpp src/FSM/FSMState.cpp src/FSM/Passive_State.cpp
# src/FSM/Stand_State.cpp src/FSM/Trotting_State.cpp src/FSM/UserCmd.cpp src/FSM/Sit_Down_State.cpp src/FSM/Trotting_State_mpc.cpp
# src/FSM/State_BalanceTest.cpp src/FSM/Free_Stand_State.cpp

# src/Gait/FeetEndCal.cpp src/Gait/GaitGenerator.cpp src/Gait/WaveGenerator.cpp

# src/low/LowCmd.cpp src/low/LowState.cpp

# src/math/Kenimatics_normal_solution.cpp src/math/Reversal_solution.cpp src/math/Robot.cpp src/math/Mujoco_box.cpp
# # src/math/data_analyze.cpp

# src/sensor/Imu.cpp

# src/WBC/BalanceCtrl.cpp src/WBC/Estimator.cpp 
# # src/WBC/mujocoEstimator.cpp

# src/MPC/ConvexMPC.cpp

# src/start.cpp
# )

# # ========== 构建目标 ==========
# add_executable(${TARGET_NAME} ${SRC_FILES})

# # 头文件路径（包含LCM和GLib的所有必要路径）
# target_include_directories(${TARGET_NAME} PRIVATE
#     ${PROJECT_SOURCE_DIR}/include

#     /usr/local/aarch64-linux-gnu/include
#     /usr/aarch64-linux-gnu/include

#     /usr/include/glib-2.0
#     /usr/lib/aarch64-linux-gnu/glib-2.0/include
#     PRIVATE ${LGPIO_INCLUDE_DIR}
# )

# # 链接库（手动添加LCM及其所有依赖，完全绕过pkg-config）
# target_link_libraries(${TARGET_NAME} PRIVATE
#     pthread
#     /usr/local/aarch64-linux-gnu/lib/libcserialport.so 
#     /usr/aarch64-linux-gnu/lib/libquadprog.a

#     osqp::osqpstatic m dl rt
    
#     # /usr/lib/aarch64-linux-gnu/libcurl.so
#     /usr/local/aarch64-linux-gnu/lib/liblcm.a
#     /usr/lib/aarch64-linux-gnu/libgthread-2.0.so
#     /usr/lib/aarch64-linux-gnu/libglib-2.0.so
#     PRIVATE ${LGPIO_LIBRARY}
# )

# # ========== 自定义部署目标 ==========
# add_custom_target(deploy
#     COMMAND echo "===== 传输文件到 ${BOARD_USER}@${BOARD_IP} ====="
#     COMMAND scp ${TARGET_NAME} ${BOARD_USER}@${BOARD_IP}:~/
#     COMMAND echo "===== 传输成功 ====="
#     COMMAND echo "下一步操作："
#     COMMAND echo "  1. ssh ${BOARD_USER}@${BOARD_IP}"
#     COMMAND echo "  2. chmod +x ${TARGET_NAME}"
#     COMMAND echo "  3. ./${TARGET_NAME}"
#     DEPENDS ${TARGET_NAME}
#     COMMENT "Deploying to target board..."
# )