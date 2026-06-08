#ifndef TROTTING_STATE_H
#define TROTTING_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "Gait/GaitGenerator.h"
#include "Reversal_solution.h"

class Trotting_State : public FSMState{
public:
    Trotting_State(ControlComponent * ctrlComp);
    ~Trotting_State();
    void enter();
    void run();
    void exit();
    virtual FSMStateName CheckChange();
    void setHighCmd(double vx, double vy, double wz);

private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    Vec3    _posBody, _velBody;
    double  _yaw, _dYaw;
    Vec34   _posFeetGlobal, _velFeetGlobal;                 // 足端位置  足端速度
    Vec34   _posFeet2BGlobal;                               // 足端位置+质心位置
    RotMat  _B2G_RotMat, _G2B_RotMat;
    Vec12    _q;
    Eigen::Matrix<float,12,1>   _qqq;
    Eigen::Matrix<float,12,1>   _www;

    Vec3    _pcd;                                           // 存储xyz上的位移
    Vec3    _vCmdGlobal, _vCmdBody;
    double  _yawCmd, _dYawCmd;                              // _yawCmd:累积转过的角度  _dYawCmd:遥控器控制的期望转动速度
    double  _dYawCmdPast;
    double  _vxCmdPast;
    double  _vyCmdPast;
    Vec3    _wCmdGlobal;
    Vec34   _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34   _posFeet2BGoal, _velFeet2BGoal;
    RotMat  _Rd;                                            // 存储当前旋转矩阵
    Vec3    _ddPcd, _dWbd;
    Vec34   _forceFeetGlobal, _forceFeetBody;
    Vec34   _qGoal, _qdGoal;
    Vec12   _tau;

    double  _gaitHeight;
    Vec3    _posError, _velError;

    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;

    Vec2    _vxLim, _vyLim, _wyawLim;
    Vec4 *  _phase;
    VecInt4 *_contact;
    Vec4 _userValue;
    LowState* _lowstate;

    Eigen::Matrix<float,3,3>     _KP;
    Eigen::Matrix<float,3,3>     _KD;
};
#endif