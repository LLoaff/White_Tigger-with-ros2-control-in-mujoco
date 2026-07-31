#ifndef FREE_STATE_H
#define FREE_STATE_H

#include "FSM/ControlComponent.h"
#include "FSM/FSMState.h"
#include "FSM/EnumClassList.h"
#include  "math/Kenimatics_normal_solution.h"
class Free_State : public FSMState
{
public:
    Free_State(ControlComponent * free_ctrl_comp);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();
private:

};
#endif