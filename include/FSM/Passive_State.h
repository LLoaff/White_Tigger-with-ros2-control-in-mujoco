#ifndef PASSIVE_STATE_H
#define PASSIVE_STATE_H

#include "FSM/ControlComponent.h"
#include "FSM/FSMState.h"
#include "FSM/EnumClassList.h"
#include "math/Kenimatics_normal_solution.h"
#include "math/mathtool.h"

class Passive_State : public FSMState
{
public:
    Passive_State(ControlComponent * passive_ctrl_comp);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();

};
#endif