#ifndef LOWSTATE_H
#define LOWSTATE_H

#include "dm_SDK/damiao.h"
#include <syslog.h>
#include <math.h>
#include "Imu.h"
#include <eigen3/Eigen/Dense>
struct Motor_State {
    uint8_t id;
    float q;
    float dq;
    float tau;
    float kp;
    float kd;
};
struct Angle_Initialization     // 手动标定电机角度
{
  float fr_hip_joint = -0.2;
  float fr_thigh_joint = 0.98;
  float fr_calf_joint = -2.64;

  float fl_hip_joint = 0.2;
  float fl_thigh_joint = 0.98;
  float fl_calf_joint = -2.64;

  float br_hip_joint = -0.2;
  float br_thigh_joint = 0.98;
  float br_calf_joint = -2.64;

  float bl_hip_joint = 0.2;
  float bl_thigh_joint = 0.98;
  float bl_calf_joint = -2.64;
};

class LowState
{
public:
  
    LowState();

    damiao::Motor   _motor_data[12];// 接受电机原始数据
    struct Angle_Initialization Angle_Initialization_Variable;
    Imu         _imu;
    struct Motor_State _motor_state[12];
};


#endif