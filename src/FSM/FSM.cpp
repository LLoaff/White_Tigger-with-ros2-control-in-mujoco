#include "FSM/FSM.h"
FSM::FSM(ControlComponent *_ctrlcomp):_fsm_ctrl(_ctrlcomp){
    _fsm_state_list.invalid     = nullptr;
    _fsm_state_list.passive     = new Passive_State(_ctrlcomp);
    _fsm_state_list.free        = new Free_State(_ctrlcomp);
    _fsm_state_list.stand       = new Stand_State(_ctrlcomp);
    _fsm_state_list.free_stand  = new Free_Stand_State(_ctrlcomp);
    _fsm_state_list.balance     = new Balance_State(_ctrlcomp,&_fsm_ctrl->_ioros->_state);
    _fsm_state_list.trotting    = new Trotting_State(_ctrlcomp);
    _fsm_state_list.sit_down    = new Sit_Down_State(_ctrlcomp);
    initialize();
}

void FSM::initialize(){
    _current_state = _fsm_state_list.passive;
    _current_state->enter();
    _next_state = _current_state;
    _mode = FSMMode::NORMAL;
}

void FSM::run(){
    _start_time = getSystemTime();
    _fsm_ctrl->_ioros->Update();        // 对电机发送命令
    _fsm_ctrl->runWaveGen();
    // if(_fsm_ctrl->user_cmd->_turn == 1)
    //     _fsm_ctrl->_servo->setAngle(0);
    // else if (_fsm_ctrl->user_cmd->_turn == 2)
    //     _fsm_ctrl->_servo->setAngle(180);
    
    // _fsm_ctrl->_estimator->run();
    if(_mode == FSMMode::NORMAL)
    {
        _current_state->run();          // 当前 状态执行一次run 

        _next_state_name = _current_state->CheckChange();

        if(_next_state_name != _current_state->_state_name)
        {   

            _mode = FSMMode::CHANGE;
            
            _next_state = GetNextState(_next_state_name);       // GetNextState函数会根据_next_state_name的enum值，返回 _fsm_state_list下不同的地址
            
        }
    }

    else if(_mode == FSMMode::CHANGE)
    {
        _current_state->exit();
        _current_state = _next_state;
        _current_state->enter();

        _mode = FSMMode::NORMAL;
        _current_state->run();
    }
    absoluteWait(_start_time, (long long)(_fsm_ctrl->dt * 1000000));       // dt 需初始化时手动赋值
}

FSMState* FSM::GetNextState(FSMStateName fsm_state_name){

    switch (fsm_state_name)
    {
        case FSMStateName::INVALID:
            return _fsm_state_list.invalid;
            break;
        case FSMStateName::PASSIVE:
            return _fsm_state_list.passive;
            break;
        case FSMStateName::FREE:
            return _fsm_state_list.free;
            break;
        case FSMStateName::STAND:
            return _fsm_state_list.stand;
            break;
        case FSMStateName::FREE_STAND:
            return _fsm_state_list.free_stand;
            break;
        case FSMStateName::BALANCE:
            return _fsm_state_list.balance;
            break;
        case FSMStateName::TROTTING:
            return _fsm_state_list.trotting;
            break;
        case FSMStateName::SIT_DOWN:
            return _fsm_state_list.sit_down;
            break;
        default:
            return _fsm_state_list.invalid;
            break;
    }
}

FSM::~FSM()
{
    delete _fsm_state_list.invalid;
    delete _fsm_state_list.passive;
    delete _fsm_state_list.free;
    delete _fsm_state_list.stand;
    delete _fsm_state_list.free_stand;
    delete _fsm_state_list.balance;
    delete _fsm_state_list.trotting;
    delete _fsm_state_list.sit_down;
}