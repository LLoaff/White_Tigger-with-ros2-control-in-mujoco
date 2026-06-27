#include "FSM/Sit_Down_State.h"

Sit_Down_State::Sit_Down_State(ControlComponent * sit_down_ctrl_comp):FSMState(sit_down_ctrl_comp,FSMStateName::SIT_DOWN,"sit_down"){

}

void Sit_Down_State::enter()
{
    Eigen::Matrix<float,3,1> dq,kp,kd,tau,speed;

    dq<< 0, 0, 0;
    tau<< 0 , 0 ,0;

    kp<< 31 , 35 , 37;
    kd<< 5 , 5 , 5;

    _target_angle(0) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.fr_hip_joint;
    _target_angle(1) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.fr_thigh_joint;
    _target_angle(2) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.fr_calf_joint;
    _target_angle(3) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.fl_hip_joint;
    _target_angle(4) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.fl_thigh_joint;
    _target_angle(5) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.fl_calf_joint;
    _target_angle(6) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.br_hip_joint;
    _target_angle(7) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.br_thigh_joint;
    _target_angle(8) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.br_calf_joint;
    _target_angle(9) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.bl_hip_joint;
    _target_angle(10) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.bl_thigh_joint;
    _target_angle(11) = _fstate_ctrl->_ioros->_state.Angle_Initialization_Variable.bl_calf_joint;
    
    for(int i=0;i<4;i++)
    {   

        _fstate_ctrl->_ioros->SetP(i,kp);
        _fstate_ctrl->_ioros->SetD(i,kd);
        _fstate_ctrl->_ioros->SetDq(i,dq);
        _fstate_ctrl->_ioros->SetTau(i,tau);
        _start_angle(3*i+0)  =  _fstate_ctrl->_ioros->_state._motor_state[3*i+0].q;
        _start_angle(3*i+1)  =  _fstate_ctrl->_ioros->_state._motor_state[3*i+1].q;
        _start_angle(3*i+2)  =  _fstate_ctrl->_ioros->_state._motor_state[3*i+2].q;
    }
    // std::cout<< "_target_angle: \n" << _target_angle<< std::endl;
    _fstate_ctrl->setAllStance();

   std::cout<<"sit_down"<<std::endl;     

}

void Sit_Down_State::run(){
    Eigen::Matrix<float,12,1> target_q;
    // 线性插值算法
    _percent += (float)1/_duration;
    _percent = _percent>1 ? 1 :  _percent;

    target_q = (1-_percent)*_start_angle + _percent*_target_angle;
    
    // std::cout<<"q: \n"<< _fstate_ctrl->_ioros->getQ12() <<std::endl;
    
    // if(_percent != 1){
    //     std::cout<< "target_q:\n"<< target_q <<"---\n"<<std::endl;
    // }
    // std::cout<< "target_q\n"<< target_q <<std::endl;
    _fstate_ctrl->_ioros->SetQ(target_q);
    // std::cout<< "tau:\n"<< tau <<"---"<<std::endl;
    // std::cout<< "stand"<<std::endl;
    // auto r = rotMatToRPY (_fstate_ctrl->_ioros->_state._imu.GetRotMat());
      
}

void Sit_Down_State::exit(){
    _percent = 0;
}

FSMStateName Sit_Down_State::CheckChange(){
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
    if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;
    else if ( user == UserValue::FREE)
        return FSMStateName::FREE;
    else if( user == UserValue::STAND)
        return FSMStateName::STAND;   
    return FSMStateName::SIT_DOWN;
}