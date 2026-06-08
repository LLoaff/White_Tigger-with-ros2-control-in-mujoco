#ifndef SERVO_H
#define SERVO_H

#include <iostream>
#include <vector>
#include <unistd.h>
#include <lgpio.h>


/**
 * @brief 舵机控制类（基于lgpio v0.2.2库，树莓派5B专属）
 * 完全适配v0.2.2版本API，经过编译测试验证
 */
class Servo {
private:
    int gpioHandle;       // lgpio芯片句柄
    int servoPin;         // 舵机信号线引脚（BCM编码）
    int minPulseWidth;    // 0度对应的脉冲宽度(微秒)
    int maxPulseWidth;    // 180度对应的脉冲宽度(微秒)
    int minAngle;         // 舵机最小角度
    int maxAngle;         // 舵机最大角度
    int currentAngle;     // 当前角度

public:
    /**
     * @brief 构造函数
     * @param pin 舵机信号线引脚（BCM编码）
     * @param minPulse 0度脉冲宽度，默认500μs
     * @param maxPulse 180度脉冲宽度，默认2500μs
     * @param minAng 最小角度，默认0度
     * @param maxAng 最大角度，默认180度
     */
    Servo(int pin, int minPulse=500, int maxPulse=2500, int minAng=0, int maxAng=180);

    /**
     * @brief 析构函数
     */
    ~Servo();

    /**
     * @brief 将角度转换为脉冲宽度
     * @param angle 目标角度
     * @return 对应的脉冲宽度(微秒)
     */
    int angleToPulse(int angle);

    /**
     * @brief 设置舵机到指定角度
     * @param angle 目标角度
     * @param duration 转动持续时间(毫秒)，0表示最快速度
     * @return 成功返回0，失败返回-1
     */
    int setAngle(int angle, int duration=0);

    /**
     * @brief 停止舵机PWM输出
     * @return 成功返回0，失败返回-1
     */
    int stop();

    /**
     * @brief 获取当前角度
     * @return 当前角度
     */
    int getCurrentAngle();

    /**
     * @brief 校准舵机脉冲宽度范围
     * @param minPulse 新的最小脉冲宽度
     * @param maxPulse 新的最大脉冲宽度
     */
    void calibrate(int minPulse, int maxPulse);
};
#endif