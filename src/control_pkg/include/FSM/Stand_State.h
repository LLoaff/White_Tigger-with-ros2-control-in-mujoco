#ifndef STAND_STATE_H
#define STAND_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "Reversal_solution.h"

class Stand_State : public FSMState
{
public:
    Stand_State(ControlComponent * stand_ctrl_comp);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();

private:
    Eigen::Matrix<float,12,1>    _target_xyz;
    Eigen::Matrix<float,12,1>    _start_xyz;
    Eigen::Matrix<float,12,1>    _target_speed;
    Eigen::Matrix<float,12,1>    _target_angle;
    Eigen::Matrix<float,12,1>    _start_angle;
    Eigen::Matrix<int,4,1>*      _conact;
    Eigen::Matrix<float,3,3>     _KP;
    Eigen::Matrix<float,3,3>     _KD;
    float                        _duration = 500;
    float                        _percent  = 0;
};
#endif