#include "FSM/Stand_State.h"

Stand_State::Stand_State(ControlComponent * stand_ctrl_comp):FSMState(stand_ctrl_comp,FSMStateName::STAND,"stand")
,_conact(stand_ctrl_comp->_contact){}

void Stand_State::enter()
{
    Eigen::Matrix<double,3,1> dq,kp,kd,tau,speed;

    dq<< 0, 0, 0;
    speed<< 0,0,0;
    tau<< 0 , 0 ,0;
    if(use_go1_model == 1){
        _target_xyz <<   0,-0.-0.08325,-0.3200,
                         0,0.08325,-0.3200,
                        -0,-0.08325,-0.3200,
                        -0,0.08325,-0.3200;
        kp<< 70.0 , 70.0 , 70;
        kd<< 7.2 , 7.2 , 7.2;

        _KP<< 70.0,  0 ,   0,
            0 ,   70.0,  0,
            0 ,   0 ,   70.0;

        _KD<< 8.0, 0,    0,
            0,   8.0,  0,
            0,   0,   8.0;
    }
    else{
        _target_xyz << 0,-0.087,-0.21,
                       0, 0.087,-0.21,
                       0,-0.087,-0.21,
                       0, 0.087,-0.21;
        kp<< 14.0 , 14.0 , 14;
        kd<< 1.2 , 1.2 , 1.2;

        _KP<< 45.0,  0 ,   0,
            0 ,   45.0,  0,
            0 ,   0 ,   55.0;

        _KD<< 8.0, 0,    0,
            0,   8.0,  0,
            0,   0,   8.0;
    }
    




    
    for(int i=0;i<4;i++)
    {   

        _fstate_ctrl->_ioros->SetP(i,kp);
        _fstate_ctrl->_ioros->SetD(i,kd);
        _fstate_ctrl->_ioros->SetDq(i,dq);
        _fstate_ctrl->_ioros->SetTau(i,tau);
        _target_speed.segment(3*i,3) = speed;
        _target_angle.segment(3*i,3) =  Reversal_Solution_Update(i,_target_xyz(3*i+0),_target_xyz(3*i+1),_target_xyz(3*i+2));
        _start_angle(3*i+0)  =  _fstate_ctrl->_ioros->_state->_motor_state[3*i+0].q;
        _start_angle(3*i+1)  =  _fstate_ctrl->_ioros->_state->_motor_state[3*i+1].q;
        _start_angle(3*i+2)  =  _fstate_ctrl->_ioros->_state->_motor_state[3*i+2].q;
    }
    // std::cout<< "_target_angle: \n" << _target_angle<< std::endl;
    // _target_angle = Reversal_GetQ(_target_xyz.cast<double>(),FrameType::BODY).cast<float>();

    _start_xyz =vec34ToVec12(GetFeetPos2BODY(*_fstate_ctrl->_ioros->_state,FrameType::HIP));
    
    _fstate_ctrl->setAllStance();
   std::cout<<"stand"<<std::endl;     

}

void Stand_State::run(){
    Eigen::Matrix<double,12,1> target_q;
    Eigen::Matrix<double,12,1> q;
    Eigen::Matrix<double,12,1> pos;
    Eigen::Matrix<double,12,1> tau;
    Eigen::Matrix<double,12,1> w;

    // 线性插值算法
    _percent += (double)1/_duration;
    _percent = _percent>1 ? 1 :  _percent;

    w = _fstate_ctrl->_ioros->getW12();
    q = _fstate_ctrl->_ioros->getQ12();
    pos= (1-_percent)*_start_xyz + _percent*_target_xyz;
    target_q = (1-_percent)*_start_angle + _percent*_target_angle;
    
    tau =CalTaus(q,w,_KP,_KD,_target_xyz,_target_speed,FrameType::HIP);
   
    // std::cout<<"q: \n"<< _fstate_ctrl->_ioros->getQ12() <<std::endl;
    
    // if(_percent != 1){
    //     std::cout<< "target_q:\n"<< target_q <<"---\n"<<std::endl;
    // }
    // std::cout<< "target_q\n"<< target_q <<std::endl;
    
    _fstate_ctrl->_ioros->SetQ(target_q);
    _fstate_ctrl->_ioros->SetTau(tau);
    // std::cout<< "tau:\n"<< tau <<"---"<<std::endl;
    // std::cout<< "stand"<<std::endl;
    // auto r = rotMatToRPY (_fstate_ctrl->_ioros->_state._imu.GetRotMat());
      
}

void Stand_State::exit(){
    _percent = 0;
}

FSMStateName Stand_State::CheckChange(){
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
    if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;
    else if ( user == UserValue::TROTTING)
        return FSMStateName::TROTTING;
    else if ( user == UserValue::SIT_DOWN)
        return FSMStateName::SIT_DOWN;
    else if ( user == UserValue::BALANCE)
        return FSMStateName::BALANCE;
    return FSMStateName::STAND;
}