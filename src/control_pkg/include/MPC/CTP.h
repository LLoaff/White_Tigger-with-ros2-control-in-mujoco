#ifndef CTP_H
#define CTP_H

#include "FSM/UserCmd.h"
#include "Imu.h"
#include "mathtool.h"
#include "Estimator.h"
#include <vector>

class CTP{
public:
    CTP(Estimator* _est,Eigen::Matrix<double, 3, 4>& footpos, float& _user_vx_B, float& _user_vy_B, float& _user_vz_B);
    void update();
    Eigen::VectorXd * getA_addr();
    std::vector<Eigen::VectorXd> desirex;
    
    Eigen::MatrixXd D;
private:
    Imu* _imu;
    Estimator * _est;
    double _dt;  // mpc 的控制周期
    // 在机身坐标下的期望速度
    double _d_vx_B;
    double _d_vy_B;
    double _d_wz_B;

    float& _user_vx_B;
    float& _user_vy_B;
    float& _user_vz_B;

    double _d_vx_O;
    double _d_vy_O;
    double _d_wz_O;

    double _yaw;  // 此时的朝向
    double _d_yaw; // 期望的朝向
    double _d_roll; 
    double _d_pitch; 
    double _d_high=0.21;
    
    Eigen::Matrix<double,3,1> _d_Vcom_B;
    Eigen::Matrix<double,3,1> _d_Vcom_O;
    Eigen::Matrix<double,3,1> _d_Wcom_O;
    // Eigen::Matrix<float,3,1> _Pcom_B;
    Eigen::Matrix<double,3,1> _Pcom_O;
    Eigen::Matrix<double,3,1> _d_Pcom_O;
    
    Eigen::MatrixXd W;
    Eigen::VectorXd Z, A, _A, N, N_;
    Eigen::Vector3d Tao, dFai;
    Eigen::Matrix<double, 3, 4> _footPosGlobal;
    float w;  // 堵转保护系数， 越大，抵抗外力改变位置能力越强
 };

#endif