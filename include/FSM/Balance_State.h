#ifndef BALANCE_STATE_H
#define BALANCE_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "BalanceCtrl.h"
#include "Estimator.h"
#include <eigen3/Eigen/Dense>
#include "mathtool.h"
#include "mathTypes.h"

class Balance_State: public FSMState
{
public:
    Balance_State(ControlComponent * balance_ctrl_comp,LowState* lowstate);
    ~Balance_State();
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();
private:
    void calcTau();

    Estimator *_est;
    BalanceCtrl *_balCtrl;
    Eigen::Matrix<int,4,1> *_contact;
    LowState*               _lowstate;

    Eigen::Matrix<double,3,3> _Rd, _RdInit;                     // 旋转矩阵
    Eigen::Matrix<double,3,1> _pcd, _pcdInit;                   // _pcd：目标质心位置  _pcdInit：进入该模式后记录第一次质心位置
    double _kpw;                                                // 角度误差的比例系数
    Eigen::Matrix<double,3,3> _Kpp, _Kdp, _Kdw;                 // _Kpp：位置误差的比例矩阵   _Kdp：速度误差的微分矩阵   _Kdw：角速度误差的微分矩阵
    Eigen::Matrix<double,3,1> _ddPcd, _dWbd;                    // _ddPcd：期望加速度      _dWbd：期望角加速度

    Eigen::Matrix<float,12,1>  _q;                              // _q：电机角度
    Eigen::Matrix<double,12,1> _tau;                            // _tau：生成力矩
    Eigen::Matrix<double,3,1> _posBody, _velBody;               //_posBody：机身位置     _velBody：机身速度
    Eigen::Matrix<double,3,3> _B2G_RotMat, _G2B_RotMat;         //旋转矩阵 _B2G_RotMat：Rgb     _G2B_RotMat：Rbg
    Eigen::Matrix<double,3,4> _posFeet2BGlobal;                 //所有腿在 相对于机身的global系下的位置
    Eigen::Matrix<double,3,4> _forceFeetGlobal, _forceFeetBody; 

    float _xMax, _xMin;
    float _yMax, _yMin;
    float _zMax, _zMin;
    float _yawMax, _yawMin;
};
#endif