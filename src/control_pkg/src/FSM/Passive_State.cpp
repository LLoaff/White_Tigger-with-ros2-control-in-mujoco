#include "FSM/Passive_State.h"



Passive_State::Passive_State(ControlComponent * passive_ctrl_comp):FSMState(passive_ctrl_comp,FSMStateName::PASSIVE,"Passive"){}

void Passive_State::enter()
{
    // Eigen::Matrix<float,3,1> p;
    // p<< 150,150,150;
    // Eigen::Matrix<float,3,1> d;
    // d<< 20 , 20 , 20;
    // Eigen::Matrix<float,12,1> q;
    // for(int i(0);i<12;++i){
    //     q(i) = _fstate_ctrl->_ioros->_init_q(i);
    // }

    Eigen::Matrix<float,3,1> d;
    d<< 0.8 , 0.8 , 0.8;
    for(int i=0;i<4;i++)
    {
        _fstate_ctrl->_ioros->SetD(i,d);
        _fstate_ctrl->_ioros->SetZeroDq();
        _fstate_ctrl->_ioros->SetZeroTau();
        _fstate_ctrl->_ioros->SetZeroP();
    }

    _fstate_ctrl->setAllStance();
   std::cout<<"passive"<<std::endl;     
}

void Passive_State::run(){
//    std::cout<<"q0: "<<_fstate_ctrl->_ioros->_state._imu.quaternion[0]<<" q1: "<<_fstate_ctrl->_ioros->_state._imu.quaternion[1]<<
//    " q2: "<<_fstate_ctrl->_ioros->_state._imu.quaternion[2]<<" q3: "<<_fstate_ctrl->_ioros->_state._imu.quaternion[3]<<std::endl;     
//    std::cout<<"q: \n"<< _fstate_ctrl->_ioros->getQ12() <<std::endl;     

}
void Passive_State::exit(){
}

FSMStateName Passive_State::CheckChange()
{
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();

    if( user == UserValue::STAND )
        return FSMStateName::STAND ;
    else if(user == UserValue::FREE)
        return FSMStateName::FREE;
    else if(user == UserValue::SIT_DOWN)
        return FSMStateName::SIT_DOWN;
    // else if(user == UserValue::BALANCE)
    //     return FSMStateName::BALANCE;
    return FSMStateName::PASSIVE;
}