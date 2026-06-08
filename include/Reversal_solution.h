#ifndef REVERSAL_SOLUTION_H
#define REVERSAL_SOLUTION_H

#include <eigen3/Eigen/Dense> // 稠密矩阵头文件
#include <math.h> 
#include "mathTypes.h"
#include "mathtool.h"
#include "Kenimatics_normal_solution.h"

inline float clamp(float val, float min_val, float max_val);
float q1_ik(float py, float pz, float l1);
float q3_ik(float b3z, float b4z, float b);
float q2_ik(float q1, float q3, float px, float py, float pz, float b3z, float b4z);
Eigen::Matrix<float,3,1> Reversal_Solution_Update(uint8_t group , float x , float y , float z);
Eigen::Matrix<float,3,1> Reversal_Update_B(uint8_t group , float x , float y , float z);
Vec12 Reversal_GetQ(const Vec34 &vecP, FrameType frame);
// Eigen::Matrix<double,3,3> calcJaco(int legid , Eigen::Matrix<float,3,1> q);
Vec3 calcQd(int legid,Vec3 pEe, Vec3 vEe, FrameType frame);
Vec12 Reversal_GetQd(const Vec34 &pos, const Vec34 &vel, FrameType frame);

#endif