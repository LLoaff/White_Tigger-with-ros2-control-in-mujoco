#ifndef TROTTING_STATE_H
#define TROTTING_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
// #include "Gait/GaitGenerator.h"
#include "Reversal_solution.h"
#include "Gait/FeetEndCal.h"

#include <lcm/lcm-cpp.hpp>
#include "plot.hpp"

class Trotting_State : public FSMState{
public:
    Trotting_State(ControlComponent * ctrlComp);
    ~Trotting_State();
    void enter();
    void run();
    void exit();
    virtual FSMStateName CheckChange();

private:
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    bool checkStepOrNot();
    void sendPlot(float x,float y,float z);
    Estimator *_est;

    double  _yaw, _dYaw;
    Vec34   _posFeetGlobal, _velFeetGlobal;                 // 足端位置  足端速度
    Vec12    _q;
    Eigen::Matrix<float,12,1>   _qqq;
    Eigen::Matrix<float,12,1>   _www;

    // Vec3    _pcd;                                           // 存储xyz上的位移
    Vec3    _vCmdGlobal, _vCmdBody;
    double  _yawCmd, _dYawCmd;                              // _yawCmd:累积转过的角度  _dYawCmd:遥控器控制的期望转动速度
    double  _dYawCmdPast;
    double  _vxCmdPast;
    double  _vyCmdPast;
    Vec3    _wCmdGlobal;
    Vec34   _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34   _qGoal, _qdGoal;
    Vec12   _tau;

    double  _gaitHeight;
    Vec3    _posError, _velError;

    Vec2    _vxLim, _vyLim, _wyawLim;
    Vec4 *  _phase;
    VecInt4 *_contact;
    Vec4 _userValue;
    LowState* _lowstate;

    Eigen::Matrix<float,3,3>     _KP;
    Eigen::Matrix<float,3,3>     _KD;

    FeetEndCal *_feetCal;

    lcm::LCM *          _lcm;
    float               x,y,z;
    plot_msg_cpp::plot  _msg;
};
#endif