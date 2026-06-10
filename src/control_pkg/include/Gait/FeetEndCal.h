#ifndef FEETENDCAL_H
#define FEETENDCAL_H

// #include "FSM/ControlComponent.h"
#include "low/LowState.h"
#include "mathTypes.h"
#include "Robot.h"
#include "Estimator.h"
#include "low/LowState.h"
#include "Gait/WaveGenerator.h"
#include "MPC/CTP.h"

/*计算落脚点*/
class FeetEndCal{
public:
    FeetEndCal(Estimator* est,LowState* lowstate,WaveGenerator* wave,CTP* ctp,Eigen::Matrix<int,4,1>& contact);
    void cal(Eigen::Matrix<double,3,4>& FootdesirePos,Eigen::Matrix<double,3,4>& FootdesireVelocity,Eigen::Vector3d _dvO,double _dwz);

    Eigen::Matrix<double,3,4> _FootdesirePos,_FootdesireVelocity;
private:
    LowState *_lowState;
    Estimator *_est;

    float off_set;
    Eigen::Vector3d SymPb1,SymPb2,SymPb3,SymPb4;
    std::vector<Eigen::Vector3d> SymPb;
    std::vector<Eigen::Vector3d> Pcomtouch; // 触地时的pcom
    Eigen::Vector4d              FaiZtouch;
    std::vector<Eigen::Vector3d> Psymtouch; // 触地时的phip
    std::vector<Eigen::Vector3d> P1;
    std::vector<Eigen::Vector3d> P2;
    std::vector<Eigen::Vector3d> P3;
    std::vector<Eigen::Vector3d> P4;

    Eigen::Matrix<double,3,4>& _footpos_Global;
    std::vector<Eigen::Vector3d> Pswend;
    std::vector<Eigen::Vector3d> Pstend;
    std::vector<Eigen::Vector3d> Pstsw;
    // std::vector<Eigen::Vector3d> FootdesirePos;
    // std::vector<Eigen::Vector3d> FootdesireVelocity;


    float _Tstance, _Tswing;
    Eigen::Vector3d _d_V_O;
    float           _yaw;
    float           _d_wz_B;
    float kp;
    const Eigen::Vector4d&  _tsw;
    VecInt4& _contact;
    Eigen::VectorXd *       A;
    double dfooth; // 期望抬腿高度

};
#endif  // FEETENDCAL_H