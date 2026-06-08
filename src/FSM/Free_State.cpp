
#include "FSM/Free_State.h"

Free_State::Free_State(ControlComponent * free_ctrl_comp):FSMState(free_ctrl_comp,FSMStateName::FREE,"free"){}

void Free_State::enter()
{
    Eigen::Matrix<float,3,1> d;
    d<< 0.0 , 0.0 , 0.0;

    for(int i=0;i<4;i++)
    {
        _fstate_ctrl->_ioros->SetD(i,d);   
        _fstate_ctrl->_ioros->SetZeroDq(i);
        _fstate_ctrl->_ioros->SetZeroTau(i);
        _fstate_ctrl->_ioros->SetZeroP();
    }
    _fstate_ctrl->setAllSwing();
   std::cout<<"free"<<std::endl;     

}

void Free_State::run()
{

    //std::cout<<"q: \n"<< _fstate_ctrl->_ioros->getQ12() <<std::endl;
    // Eigen::Matrix<float,12,1> q = _fstate_ctrl->_ioros->getQ12();
    // for(int i=0;i<4;i++){
    //     std::cout<<"leg"<<i<<": \n"<<GetPos_H(i,q(3*i+0),q(3*i+1),q(3*i+2))<<std::endl;
    // }
}

void Free_State::exit()
{

}

FSMStateName Free_State::CheckChange()
{
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
    if( user == UserValue::STAND )
        return FSMStateName::STAND ;
    else if(user == UserValue::FREE)
        return FSMStateName::FREE;
    else if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;

    return FSMStateName::FREE;
}
