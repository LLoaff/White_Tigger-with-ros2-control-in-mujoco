#ifndef KENIMATICS_NORMAL_SOLUTION_H
#define KENIMATICS_NORMAL_SOLUTION_H

#include <eigen3/Eigen/Dense> // 稠密矩阵头文件
#include <math.h> 
#include "mathtool.h"
#include "FSM/EnumClassList.h"
#include "low/LowState.h"
#include <iostream>

                            // group用来区别是第几条腿
Eigen::Matrix<float,3,1> GetPos_H(uint8_t group, float theta1, float theta2, float theta3);

Eigen::Matrix<float,3,1> GetPos_B(uint8_t group, float theta1, float theta2, float theta3);

Eigen::Matrix<float,3,1> GetPos_S(Eigen::Matrix<float,3,1> init_pos,uint8_t id,float theta1, float theta2, float theta3);
Eigen::Matrix<float,3,1> Pos_Speed(int group,float theta1, float theta2, float theta3 , float th1_s , float th2_s , float th3_s);

// 计算以机身为全局坐标系的
Eigen::Matrix<double,3,4> GetFeetPos2BODY(LowState &lowstate , FrameType frame);

Eigen::Matrix<double,3,1> GetFeetPos2BODY(LowState &lowstate,int i , FrameType frame);

Eigen::Matrix<double,3,4> GetFeetSpeed2BODY(LowState &lowstate , FrameType frame);
/* tau */
Eigen::Matrix<float,3,1> CalTau(
    int group,Eigen::Matrix<float,3,1> angle,Eigen::Matrix<float,3,1> w,Eigen::Matrix<float,3,3>Kp ,Eigen::Matrix<float,3,3>Kd,
    Eigen::Matrix<float,3,1> target_pos, Eigen::Matrix<float,3,1> target_speed,FrameType frame);

Eigen::Matrix<float,12,1> CalTaus(
    Eigen::Matrix<float,12,1> angle,Eigen::Matrix<float,12,1> w,Eigen::Matrix<float,3,3>Kp ,Eigen::Matrix<float,3,3>Kd,
    Eigen::Matrix<float,12,1> target_pos, Eigen::Matrix<float,12,1> target_speed,FrameType frame);

// 计算一条腿的Jaco
Eigen::Matrix<double,3,3> calcJaco(int legid , Eigen::Matrix<float,3,1> q);

//计算一条腿的Tau
Eigen::Matrix<double,3,1> calcTau(int legid ,Eigen::Matrix<float,3,1> q, Eigen::Matrix<double,3,1> force);

//获取所有腿的Tau
Eigen::Matrix<double,12,1> getTau(const Eigen::Matrix<float,12,1> &q, const Eigen::Matrix<double,3,4> feetForce);
#endif


