#include "FSM/Free_State.h"

FSMState::FSMState(ControlComponent *ctrl_comp,FSMStateName fsm_state_name,std::string state_string_name)
:_fstate_ctrl(ctrl_comp),_state_name(fsm_state_name),_state_string_name(state_string_name){

}