#ifndef SIT_DOWN_STATE_H
#define SIT_DOWN_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "Reversal_solution.h"

class Sit_Down_State : public FSMState
{
public:
    Sit_Down_State(ControlComponent * sit_down_ctrl_comp);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();

private:
    Eigen::Matrix<float,12,1>    _target_angle;
    Eigen::Matrix<float,12,1>    _start_angle;
    float                        _duration = 600;
    float                        _percent  = 0;
};
#endif