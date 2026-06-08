cmake_minimum_required(VERSION 3.8)
project(control_pkg)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LOWSTATE_DEBUG OFF)  # lowstate 数据调试
set(LOWCMD_DEBUG OFF)  # lowcmd 数据调试

if(LOWSTATE_DEBUG)
    add_definitions(-DLOWSTATE_DEBUG)
endif()
if(LOWCMD_DEBUG)
    add_definitions(-DLOWCMD_DEBUG)
endif()

add_executable(start src/start.cpp 
src/common/BalanceCtrl.cpp src/common/Estimator.cpp src/common/Imu.cpp src/common/LowPassFilter.cpp src/common/Robot.cpp  
src/FSM/Balance_State.cpp src/FSM/ControlComponent.cpp src/FSM/Free_Stand_State.cpp src/FSM/Free_State.cpp src/FSM/FSM.cpp src/FSM/FSMState.cpp
src/FSM/Passive_State.cpp src/FSM/Stand_State.cpp src/FSM/Trotting_State.cpp src/FSM/UserCmd.cpp src/FSM/Sit_Down_State.cpp
src/Gait/FeetEndCal.cpp src/Gait/GaitGenerator.cpp src/Gait/WaveGenerator.cpp
src/low/LowCmd.cpp src/low/LowState.cpp
src/math/Kenimatics_normal_solution.cpp src/math/Reversal_solution.cpp
src/damiao_SDK/damiao.cpp
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include/CSerialPort
    /usr/include/glib-2.0
    /usr/lib/x86_64-linux-gnu/glib-2.0/include
)

target_link_libraries(
start
pthread
/usr/local/lib/libcserialport.so
quadprog
/usr/lib/x86_64-linux-gnu/liblcm.so
/usr/lib/x86_64-linux-gnu/libgthread-2.0.so
/usr/lib/x86_64-linux-gnu/libglib-2.0.so
)

####################################################################################

# cmake_minimum_required(VERSION 3.8)

# # 项目名称和语言（同时声明C和C++）
# project(QuadrupedRobot CXX C)

# # ========== 交叉编译配置（只定义一次） ==========
# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR aarch64)
# set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# # 明确指定lgpio库的路径
# set(LGPIO_INCLUDE_DIR /usr/local/aarch64-linux-gnu/include)
# set(LGPIO_LIBRARY /usr/local/aarch64-linux-gnu/lib/liblgpio.so)

# # 强制交叉编译模式，不搜索主机系统路径（关键！防止找到x86_64的库）
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# # ========== C/C++标准配置 ==========
# set(CMAKE_C_STANDARD 11)
# set(CMAKE_C_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用非标准扩展

# # ========== 项目基本配置 ==========
# set(TARGET_NAME "start")
# set(BOARD_IP "192.168.8.103")
# set(BOARD_USER "shu")

# # ========== 源文件列表 ==========
# set(SRC_FILES
#     src/start.cpp
#     src/common/BalanceCtrl.cpp
#     src/common/Estimator.cpp
#     src/common/Imu.cpp
#     src/common/LowPassFilter.cpp
#     src/common/Robot.cpp
#     src/FSM/Balance_State.cpp
#     src/FSM/ControlComponent.cpp
#     src/FSM/Free_Stand_State.cpp
#     src/FSM/Free_State.cpp
#     src/FSM/FSM.cpp
#     src/FSM/FSMState.cpp
#     src/FSM/Passive_State.cpp
#     src/FSM/Stand_State.cpp
#     src/FSM/Trotting_State.cpp
#     src/FSM/UserCmd.cpp
#     src/FSM/Sit_Down_State.cpp
#     src/Gait/FeetEndCal.cpp
#     src/Gait/GaitGenerator.cpp
#     src/Gait/WaveGenerator.cpp
#     src/low/LowCmd.cpp
#     src/low/LowState.cpp
#     src/low/servo.cpp
#     src/math/Kenimatics_normal_solution.cpp
#     src/math/Reversal_solution.cpp    
#     src/damiao_SDK/damiao.cpp
# )

# # ========== 调试选项 ==========
# option(LOWSTATE_DEBUG "Enable lowstate data debug" OFF)
# option(LOWCMD_DEBUG "Enable lowcmd data debug" OFF)

# # ========== 构建目标 ==========
# add_executable(${TARGET_NAME} ${SRC_FILES})

# # 头文件路径（包含LCM和GLib的所有必要路径）
# target_include_directories(${TARGET_NAME} PRIVATE
#     ${PROJECT_SOURCE_DIR}/include
#     /usr/local/aarch64-linux-gnu/include
#     /usr/aarch64-linux-gnu/include
#     /usr/aarch64-linux-gnu/include/glib-2.0
#     /usr/aarch64-linux-gnu/lib/glib-2.0/include
#     PRIVATE ${LGPIO_INCLUDE_DIR}
# )

# # 编译定义
# if(LOWSTATE_DEBUG)
#     target_compile_definitions(${TARGET_NAME} PRIVATE LOWSTATE_DEBUG)
# endif()
# if(LOWCMD_DEBUG)
#     target_compile_definitions(${TARGET_NAME} PRIVATE LOWCMD_DEBUG)
# endif()

# # 链接库（手动添加LCM及其所有依赖，完全绕过pkg-config）
# target_link_libraries(${TARGET_NAME} PRIVATE
#     pthread
#     /usr/local/aarch64-linux-gnu/lib/libcserialport.so 
#     quadprog
#     lcm
#     gthread-2.0
#     glib-2.0
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