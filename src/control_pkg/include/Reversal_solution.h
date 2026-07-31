#ifndef REVERSAL_SOLUTION_H
#define REVERSAL_SOLUTION_H

#include <eigen3/Eigen/Dense> // 稠密矩阵头文件
#include <math.h> 
#include "mathTypes.h"
#include "mathtool.h"
#include "Kenimatics_normal_solution.h"

inline double clamp(double val, double min_val, double max_val);
double q1_ik(double py, double pz, double l1);
double q3_ik(double b3z, double b4z, double b);
double q2_ik(double q1, double q3, double px, double py, double pz, double b3z, double b4z);
Eigen::Matrix<double,3,1> Reversal_Solution_Update(uint8_t group , double x , double y , double z);
Eigen::Matrix<double,3,1> Reversal_Update_B(uint8_t group , double x , double y , double z);
Vec12 Reversal_GetQ(const Vec34 &vecP, FrameType frame);
// Eigen::Matrix<double,3,3> calcJaco(int legid , Eigen::Matrix<float,3,1> q);
Vec3 calcQd(int legid,Vec3 pEe, Vec3 vEe, FrameType frame);
Vec12 Reversal_GetQd(const Vec34 &pos, const Vec34 &vel, FrameType frame);

#endif