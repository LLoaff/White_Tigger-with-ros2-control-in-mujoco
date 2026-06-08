#ifndef FSMSTATE_H
#define FSMSTATE_H

#include "EnumClassList.h"
#include "ControlComponent.h"

class FSMState 
{
public:
    FSMState(ControlComponent *ctrl_comp,FSMStateName fsm_state_name,std::string _state_string_name);
    virtual void enter(void)=0;
    virtual void exit(void)=0;
    virtual void run(void)=0;
    virtual FSMStateName CheckChange() {return FSMStateName::INVALID;}

    FSMStateName      _state_name;
    std::string       _state_string_name;
protected:
    ControlComponent *_fstate_ctrl;
};
#endif