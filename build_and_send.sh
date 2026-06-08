
# 主控板IP地址
BOARD_IP="192.168.87.31"
# 主控板登录用户名
BOARD_USER="linaro"
# 编译输出的二进制文件名
OUTPUT_FILE="start"
INCLUDE_PATH="-I./include -I/usr/local/aarch64-linux-gnu/include -I/usr/aarch64-linux-gnu/include -I/opt/arm64-libs/include "
# 库路径（ARM64版本CSerialPort）
LIB_PATH="-L/usr/local/aarch64-linux-gnu/lib -L/usr/aarch64-linux-gnu/lib -L/opt/arm64-libs/lib"
# 链接的库
# LIBS="-lUnitreeMotorSDK_Arm64 -lpthread -lcserialport -lquadprog -llcm"
LIBS="-lUnitreeMotorSDK_Arm64 -lpthread -lcserialport -lquadprog "


LOWSTATE_DEBUG="OFF"  # lowstate 数据调试
LOWCMD_DEBUG="OFF"    # lowcmd 数据调试
SRC_FILES="src/start.cpp \
src/common/BalanceCtrl.cpp \
src/common/Estimator.cpp \
src/common/Imu.cpp \
src/common/LowPassFilter.cpp \
src/common/Robot.cpp \
src/FSM/Balance_State.cpp \
src/FSM/ControlComponent.cpp \
src/FSM/Free_Stand_State.cpp \
src/FSM/Free_State.cpp \
src/FSM/FSM.cpp \
src/FSM/FSMState.cpp \
src/FSM/Passive_State.cpp \
src/FSM/Stand_State.cpp \
src/FSM/Trotting_State.cpp \
src/FSM/UserCmd.cpp \
src/Gait/FeetEndCal.cpp \
src/Gait/GaitGenerator.cpp \
src/Gait/WaveGenerator.cpp \
src/low/LowCmd.cpp \
src/low/LowState.cpp \
src/math/Kenimatics_normal_solution.cpp \
src/math/Reversal_solution.cpp"

echo "===== 开始编译 ARM64 版本程序 ====="
echo "  - LOWSTATE_DEBUG: ${LOWSTATE_DEBUG}"
echo "  - LOWCMD_DEBUG: ${LOWCMD_DEBUG}"

# 构建编译指令
BUILD_CMD="aarch64-linux-gnu-g++ ${SRC_FILES} -o ${OUTPUT_FILE} \
-std=c++17 \
${INCLUDE_PATH} \
${LIB_PATH} \
${LIBS}"

# 根据调试开关添加宏定义
if [ "${LOWSTATE_DEBUG}" = "ON" ]; then
    BUILD_CMD="${BUILD_CMD} -DLOWSTATE_DEBUG"
fi
if [ "${LOWCMD_DEBUG}" = "ON" ]; then
    BUILD_CMD="${BUILD_CMD} -DLOWCMD_DEBUG"
fi

# 执行编译
eval ${BUILD_CMD}

# 检查编译是否成功
if [ $? -eq 0 ]; then
    echo "✅ 编译成功！生成文件：${OUTPUT_FILE}"
    
    # 验证生成文件的架构（确保是 ARM64）
    echo "👉 验证文件架构："
    file ${OUTPUT_FILE}
    
    # ===================== 传输逻辑 =====================
    echo ""
    echo "===== 开始传输文件到主控板 ${BOARD_USER}@${BOARD_IP} ====="
    scp ${OUTPUT_FILE} ${BOARD_USER}@${BOARD_IP}:~/
    
    # 检查传输是否成功
    if [ $? -eq 0 ]; then
        echo ""
        echo "✅ 文件传输成功！"
        echo "👉 下一步操作："
        echo "   1. 登录主控板：ssh ${BOARD_USER}@${BOARD_IP}"
        echo "   2. 添加执行权限：chmod +x ${OUTPUT_FILE}"
        echo "   3. 运行程序：./${OUTPUT_FILE}"
    else
        echo ""
        echo "❌ 文件传输失败！请检查："
        echo "  1. 主控板IP是否正确（当前：${BOARD_IP}）"
        echo "  2. 主控板是否开机/网络可达（ping ${BOARD_IP} 测试）"
        echo "  3. 用户名是否正确（当前：${BOARD_USER}）"
        exit 1
    fi
else
    echo ""
    echo "❌ 编译失败！请检查："
    echo "  1. 所有源文件路径是否正确"
    echo "  2. 依赖库是否已安装 ARM64 版本（特别是 /opt/arm64-libs/lib/liblcm.so）"
    echo "  3. 头文件路径是否正确"
    exit 1
fi

