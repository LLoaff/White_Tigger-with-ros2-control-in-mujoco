#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "low/LowState.h"
#include "eigen3/Eigen/Dense"
#include "mathtool.h"
#include "FSM/EnumClassList.h"
#include "Kenimatics_normal_solution.h"

class Estimator
{
public:
    Estimator(LowState* lowstate,Eigen::Matrix<int, 4, 1>* contact,Eigen::Matrix<double, 4, 1> * phase,double dt);
    void init();
    Eigen::Matrix<double, 3, 1>  getPosition();
    Eigen::Matrix<double, 3, 1>  getVelocity();
    Eigen::Matrix<double, 3, 1>  getFootPos(int i);
    Eigen::Matrix<double, 3, 4> getFeetPos();
    Eigen::Matrix<double, 3, 4> getFeetVel();
    Eigen::Matrix<double, 3, 4> getPosFeet2BGlobal();
    void run();
private:
    Eigen::Matrix<double, 18, 1>  _xhat;    // 先验x
    Eigen::Matrix<double, 3, 1>   _u;       // 输入变量 (xyz的三轴加速度)            
    Eigen::Matrix<double, 28,  1> _y;       // 输出y
    Eigen::Matrix<double, 28,  1> _yhat;    // 先验y
    Eigen::Matrix<double, 18, 18> _A;       // 状态转移矩阵
    Eigen::Matrix<double, 18, 3>  _B;       // 输入矩阵
    Eigen::Matrix<double, 28, 18> _C;       // 输出矩阵
    /*噪声*/
    Eigen::Matrix<double, 18, 18> _P;       // 预测P协方差矩阵
    Eigen::Matrix<double, 18, 18> _Ppriori; // 先验预测P协方差矩阵
    Eigen::Matrix<double, 18, 18> _Q;       // 过程噪声协方差矩阵
    Eigen::Matrix<double, 28, 28> _R;       // 测量噪声协方差矩阵
    Eigen::Matrix<double, 18, 18> _QInit;   // 过程噪声协方差矩阵 初始化
    Eigen::Matrix<double, 28, 28> _RInit;   // 测量噪声协方差矩阵 初始化
    Eigen::Matrix<double, 18, 1>  _Qdig;    // 过程噪声协方差矩阵 的对角线上的值组成的向量
    Eigen::Matrix<double, 3, 3>   _Cu;      // 输入_u的协方差矩阵

    AvgCov *                      _RCheck;
    AvgCov *                      _uCheck;
    // 输出的测量数据
    Eigen::Matrix<double, 12, 1>  _feetPos2Body;    // 腿相对与机身的位置 - 在{s}
    Eigen::Matrix<double, 12, 1>  _feetVel2Body;    // 腿相对与机身的速度 - 在{s}
    Eigen::Matrix<double,  4, 1>  _feetH;           // 腿相对与机身的高度 - 在{s}
    Eigen::Matrix<double, 28, 28> _S;               // _S = C*P*C.T + R

    Eigen::Matrix<double, 3, 3>   _rotMatB2G;       // Rsb 旋转矩阵
    Eigen::Matrix<float, 3, 1>    _g;                // 重力加速度
    LowState*                     _lowstate;
    Eigen::Matrix<double, 3, 4>   _feetPosGlobalKine, _feetVelGlobalKine;   // 运动学正解所得的 位置 、 速度
    Eigen::Matrix<double, 4, 1> * _phase;           // 步态的频率
    double                        _dt;
    double                        _largeVariance;   // 增幅
    Eigen::Matrix<int, 4, 1>*     _contact      ;   // 判断足端是否接触地面（通过力矩大小）
    double                        _trust;

    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28>> _Slu;    // _S.lu()
    Eigen::Matrix<double, 28,  1> _Sy;              // _Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<double, 28, 18> _Sc;              // _Sc = _S.inv() * C
    Eigen::Matrix<double, 28, 28> _SR;              // _SR = _S.inv() * R
    Eigen::Matrix<double, 28, 18> _STC;             // _STC = (_S.transpose()).inv() * C
    Eigen::Matrix<double, 18, 18> _IKC;             // _IKC = I - KC
};

#endif