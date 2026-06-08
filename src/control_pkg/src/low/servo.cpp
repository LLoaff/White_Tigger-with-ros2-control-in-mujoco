#include "low/servo.h"

Servo::Servo(int pin, int minPulse, int maxPulse, int minAng, int maxAng) {
    // 树莓派5B固定使用gpiochip4
    gpioHandle = lgGpiochipOpen(4);
    if (gpioHandle < 0) {
        throw std::runtime_error("无法打开GPIO芯片4，请确保以root权限运行");
    }

    servoPin = pin;
    minPulseWidth = minPulse;
    maxPulseWidth = maxPulse;
    minAngle = minAng;
    maxAngle = maxAng;
    currentAngle = -1;

    // v0.2.2正确API：使用lgGpioClaimOutput声明输出引脚
    // 参数：句柄, 标志位(0=默认), 引脚号, 初始输出值
    if (lgGpioClaimOutput(gpioHandle, 0, servoPin, 0) < 0) {
        throw std::runtime_error("无法声明GPIO引脚为输出模式");
    }
}

/**
 * @brief 析构函数
 */
Servo::~Servo() {
    // 停止PWM输出
    stop();
    // 释放GPIO引脚
    lgGpioFree(gpioHandle, servoPin);
    // 关闭GPIO芯片句柄
    if (gpioHandle >= 0) {
        lgGpiochipClose(gpioHandle);
    }
}

/**
 * @brief 将角度转换为脉冲宽度
 * @param angle 目标角度
 * @return 对应的脉冲宽度(微秒)
 */
int Servo::angleToPulse(int angle) {
    // 限制角度在有效范围内
    if (angle < minAngle) angle = minAngle;
    if (angle > maxAngle) angle = maxAngle;

    // 线性映射计算
    return minPulseWidth + 
            (maxPulseWidth - minPulseWidth) * (angle - minAngle) / (maxAngle - minAngle);
}

/**
 * @brief 设置舵机到指定角度
 * @param angle 目标角度
 * @param duration 转动持续时间(毫秒)，0表示最快速度
 * @return 成功返回0，失败返回-1
 */
int Servo::setAngle(int angle, int duration) {
    if (angle == currentAngle) return 0;

    int targetPulse = angleToPulse(angle);
    
    if (duration <= 0) {
        // 直接转到目标角度（v0.2.2正确API：6个参数）
        // 参数：句柄, 引脚, 高电平时间(μs), 低电平时间(μs), 脉冲数(0=无限), 边沿(0=上升沿开始)
        int result = lgTxPulse(gpioHandle, servoPin, targetPulse, 20000 - targetPulse, 0, 0);
        if (result == 0) {
            currentAngle = angle;
        }
        return result;
    } else {
        // 平滑转动到目标角度
        int startAngle = currentAngle;
        if (startAngle == -1) startAngle = minAngle;
        
        int steps = duration / 20; // 每20ms一步（舵机标准频率50Hz）
        float angleStep = (float)(angle - startAngle) / steps;
        
        for (int i = 0; i <= steps; i++) {
            int currentStepAngle = startAngle + (int)(angleStep * i);
            int currentStepPulse = angleToPulse(currentStepAngle);
            // v0.2.2正确API
            lgTxPulse(gpioHandle, servoPin, currentStepPulse, 20000 - currentStepPulse, 0, 0);
            usleep(20000); // 等待20ms
        }
        
        currentAngle = angle;
        return 0;
    }
}

/**
 * @brief 停止舵机PWM输出
 * @return 成功返回0，失败返回-1
 */
int Servo::stop() {
    // v0.2.2正确停止方法：发送1个0脉冲
    return lgTxPulse(gpioHandle, servoPin, 0, 0, 1, 0);
}

/**
 * @brief 获取当前角度
 * @return 当前角度
 */
int Servo::getCurrentAngle() {
    return currentAngle;
}

/**
 * @brief 校准舵机脉冲宽度范围
 * @param minPulse 新的最小脉冲宽度
 * @param maxPulse 新的最大脉冲宽度
 */
void Servo::calibrate(int minPulse, int maxPulse) {
    minPulseWidth = minPulse;
    maxPulseWidth = maxPulse;
}

