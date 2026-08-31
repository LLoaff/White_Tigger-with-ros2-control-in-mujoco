#ifndef FSM_H
#define FSM_H

#include "FSM/ControlComponent.h"
#include "FSM/EnumClassList.h"
#include "FSM/Passive_State.h"
#include "FSM/Free_State.h"
#include "FSM/Stand_State.h"
// #include "Free_Stand_State.h"
// #include "Balance_State.h"
#include "FSM/FSMState.h"
#include "math/TimeMaker.h"
#include "FSM/Trotting_State.h"
#include "FSM/Trotting_State_mpc.h"
#include "FSM/Sit_Down_State.h"
#include "math/Mujoco_box.h"
struct FSMStateList
{
    FSMState      *     invalid;
    Passive_State *     passive;
    Free_State    *     free;
    Stand_State   *     stand;
    // Free_Stand_State *  free_stand;
    // Balance_State*      balance;
    Trotting_State*     trotting;
    Trotting_State_MPC* trotting_mpc;
    Sit_Down_State*     sit_down;
};

class FSM
{
public:
    FSM(ControlComponent *_ctrlcomp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState         * GetNextState(FSMStateName fsm_state_name);
    Mujoco_box       *_mj_box;
    ControlComponent * _fsm_ctrl;
    FSMStateList       _fsm_state_list;
    FSMState         * _current_state;
    FSMState         * _next_state;
    FSMStateName       _next_state_name; // 下个状态的enum名
    FSMMode            _mode;       // 判断切换还是正常
    long long          _start_time;  
    int                _count;
};
#endif