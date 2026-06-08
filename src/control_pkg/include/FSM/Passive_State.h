#ifndef PASSIVE_STATE_H
#define PASSIVE_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "Kenimatics_normal_solution.h"
#include "mathtool.h"

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