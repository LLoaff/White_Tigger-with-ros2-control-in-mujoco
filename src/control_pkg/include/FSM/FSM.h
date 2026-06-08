#ifndef FSM_H
#define FSM_H

#include "ControlComponent.h"
#include "EnumClassList.h"
#include "Passive_State.h"
#include "Free_State.h"
#include "Stand_State.h"
#include "Free_Stand_State.h"
#include "FSMState.h"
#include "TimeMaker.h"
#include "Trotting_State.h"
#include "Sit_Down_State.h"

struct FSMStateList
{
    FSMState      *     invalid;
    Passive_State *     passive;
    Free_State    *     free;
    Stand_State   *     stand;
    Free_Stand_State *  free_stand;
    Trotting_State*     trotting;
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