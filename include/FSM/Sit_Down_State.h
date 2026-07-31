#ifndef SIT_DOWN_STATE_H
#define SIT_DOWN_STATE_H

#include "FSM/ControlComponent.h"
#include "FSM/FSMState.h"
#include "FSM/EnumClassList.h"
#include "math/Reversal_solution.h"

class Sit_Down_State : public FSMState
{
public:
    Sit_Down_State(ControlComponent * sit_down_ctrl_comp);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();

private:
    Eigen::Matrix<double,12,1>    _target_angle;
    Eigen::Matrix<double,12,1>    _start_angle;
    double                        _duration = 600;
    double                        _percent  = 0;
};
#endif