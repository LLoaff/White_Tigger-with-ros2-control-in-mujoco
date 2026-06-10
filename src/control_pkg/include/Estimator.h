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
    LowState* getLowState();
    Eigen::Vector3d getPcom();
    Eigen::Vector3d getVcom();

    Eigen::Matrix<double,3,4>   iPb;
private:

    Eigen::Matrix<double,18,18> A;
    Eigen::Matrix<double,18,3>  B;
    Eigen::Matrix<double,3,1>   U;
    Eigen::Matrix<double,28,18> H;
    Eigen::MatrixXd             Q;
    Eigen::Matrix<double,28,28> R;
    Eigen::Matrix<double,18,1> _X;
    Eigen::Matrix<double,18,1> X;
    Eigen::Matrix<double,18,18> _P;
    Eigen::Matrix<double,18,18> P;
    Eigen::Matrix<double,28,1> Z;
    Eigen::Matrix<double,18,28> K;

    Eigen::Matrix3d iden3;
    Eigen::Matrix3d _3x3;
    Eigen::Matrix<double,3,12> _3x12;
    Eigen::Matrix<double,12,3> _12x3;
    Eigen::MatrixXd             iden12;
    Eigen::MatrixXd             iden18;
    Eigen::Matrix<double, 12, 12>  _12x12 ;
    Eigen::Matrix<double, 1, 1>         Onemat;

    Eigen::Vector3d Flipb, Fripb, Rlipb, Rripb;
    Eigen::Vector3d FliPbv, FriPbv, RliPbv, RriPbv;
    Eigen::Vector3d _pcom;
    Eigen::Vector3d _vcom;

    // 输出的测量数据
    Eigen::Matrix<double, 12, 1>  _feetPos2Body;    // 腿相对与机身的位置 - 在{s}
    Eigen::Matrix<double, 12, 1>  _feetVel2Body;    // 腿相对与机身的速度 - 在{s}
    Eigen::Matrix<double,  4, 1>  _feetH;           // 腿相对与机身的高度 - 在{s}

    Eigen::Matrix<double, 3, 3>   _rotMatB2G;       // Rsb 旋转矩阵
    Eigen::Matrix<double, 3, 1>    _g;                // 重力加速度
    LowState*                     _lowstate;
    Eigen::Matrix<double, 3, 4>   _feetPosBodyKine, _feetVelGlobalKine;   // 运动学正解所得的 位置 、 速度
    Eigen::Matrix<double, 4, 1> * _phase;           // 步态的频率
    double                        _dt;
    Eigen::Matrix<int, 4, 1>*     _contact      ;   // 判断足端是否接触地面（通过力矩大小）
    double                        _trust;

};

#endif