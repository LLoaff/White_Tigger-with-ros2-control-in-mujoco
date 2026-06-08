#ifndef FREE_STATE_H
#define FREE_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
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