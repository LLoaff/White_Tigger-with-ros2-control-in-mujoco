#ifndef TROTTING_STATE_MPC_H
#define TROTTING_STATE_MPC_H

#include "FSMState.h"
#include "EnumClassList.h"
#include "math/Reversal_solution.h"
// #include "Gait/FeetEndCal.h"
#include "WBC/BalanceCtrl.h"
#include "lcm_msg_cpp/plot.hpp"
#include "Gait/GaitGenerator.h"
#include "math/Robot.h"
#include "MPC/ConvexMPC.h"

class Trotting_State_MPC : public FSMState{
public:
    Trotting_State_MPC(ControlComponent * ctrlComp);
    ~Trotting_State_MPC();
    void enter();
    void run();
    void exit();
    virtual FSMStateName CheckChange();

private:
    ConvexMPC*   _mpc;
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    bool checkStepOrNot();
    void sendPlot(float x,float y,float z);
    void sendPlot2(float x,float y,float z);
    void sendPlot3(float x,float y,float z);
    void sendPlot4(float x,float y,float z);
    Estimator *_est;
    BalanceCtrl *_balCtrl;
    GaitGenerator *_gait;
    QuadrupedRobot* _robModel;
    double  _yaw, _dYaw;
    Vec34   _posFeetGlobal, _velFeetGlobal;                 // 足端位置  足端速度
    Vec12    _q;
    Eigen::Matrix<float,12,1>   _qqq;
    Eigen::Matrix<float,12,1>   _www;


    Vec3    _vCmdGlobal, _vCmdBody;
    double  _yawCmd, _dYawCmd;                              // _yawCmd:累积转过的角度  _dYawCmd:遥控器控制的期望转动速度
    double  _dYawCmdPast;
    double  _vxCmdPast;
    double  _vyCmdPast;
    Vec3    _wCmdGlobal;
    Vec34   _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34  _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;

    Mat3 _KpSwing, _KdSwing;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;

    Vec34   _qGoal, _qdGoal;
    Vec12   _tau;

    double  _gaitHeight;

    Vec2    _vxLim, _vyLim, _wyawLim;
    Vec4 *  _phase;
    VecInt4 *_contact;
    Vec4 _userValue;
    LowState* _lowstate;

    lcm::LCM *          _lcm;
    lcm::LCM *          _lcm2;
    lcm::LCM *          _lcm3;
    lcm::LCM *          _lcm4;
    plot_msg_cpp::plot  _msg;

    Vec3 _pcd;
    RotMat _Rd;
    Vec3  _posBody, _velBody;

};
#endif
