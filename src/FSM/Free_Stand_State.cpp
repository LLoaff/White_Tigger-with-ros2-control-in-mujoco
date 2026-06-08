#include "FSM/Free_Stand_State.h"


Free_Stand_State::Free_Stand_State(ControlComponent * free_stand_ctrl_comp):FSMState(free_stand_ctrl_comp,FSMStateName::FREE_STAND,"Free_Stand"){
    _rowMax = 20 * M_PI / 180;
    _rowMin = -_rowMax;
    _pitchMax = 20 * M_PI / 180;
    _pitchMin = -_pitchMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;
    _heightMax = 0.1;
    _heightMin = -_heightMax;
}

void Free_Stand_State::enter(){

    Eigen::Matrix<float,3,1> dq,kp,b_kp,kd,tau;
    kp<< 2.0 , 2.0 , 3.5;
    b_kp<<2.0,2.0,4.2;
    kd<< 0.05 , 0.05 , 0.08;
    dq<< 0, 0, 0;
    tau<< 0 , 0 ,0;

    _fstate_ctrl->_ioros->SetP(0,kp);
    _fstate_ctrl->_ioros->SetP(1,kp);
    _fstate_ctrl->_ioros->SetP(2,b_kp);
    _fstate_ctrl->_ioros->SetP(3,b_kp);

    _pb1_pos_b = GetPos_B(0,_fstate_ctrl->_ioros->_state._motor_state[0].q ,_fstate_ctrl->_ioros->_state._motor_state[1].q,_fstate_ctrl->_ioros->_state._motor_state[2].q);

    for(int i=0;i<4;i++){
        _fstate_ctrl->_ioros->SetD(i,kd);
        _fstate_ctrl->_ioros->SetDq(i,dq);
        _fstate_ctrl->_ioros->SetTau(i,tau);

        _pos_s.col(i) = GetPos_S(_pb1_pos_b,i,_fstate_ctrl->_ioros->_state._motor_state[3*i].q,_fstate_ctrl->_ioros->_state._motor_state[3*i+1].q,_fstate_ctrl->_ioros->_state._motor_state[3*i+2].q);

    }

    _fstate_ctrl->setAllStance();
}

void Free_Stand_State::run(){
    /* 遥控归一 */
    float ly = (_fstate_ctrl->user_cmd->R_Data.ch3 - 1024) / 660.0;
    float lx = (_fstate_ctrl->user_cmd->R_Data.ch2 - 1024) / 660.0;
    float ry = (_fstate_ctrl->user_cmd->R_Data.ch1 - 1024) / 660.0;
    float rx = (_fstate_ctrl->user_cmd->R_Data.ch0 - 1024) / 660.0;
    
    Eigen::Matrix<float,3,4> q_s = Cal_PosinB(invNormalize(lx,_rowMin,_rowMax),invNormalize(ly,_pitchMin,_pitchMax),
                                            invNormalize(rx,_yawMin,_yawMax),invNormalize(ry,_heightMin,_heightMax));
    
    CalQ(q_s);                               
    // printf(" lrd:%.3f ltn:%.3f rrd:%.3f rtn:%.3f ",l_rd,l_tn,r_rd,r_tn );
    // printf(" 0:%.3f %.3f %.3f 2:%.3f %.3f %.3f ",_pos_s.col(0)(0),_pos_s.col(0)(1),_pos_s.col(0)(2),_pos_s.col(2)(0),_pos_s.col(2)(1),_pos_s.col(2)(2));
    // std::cout << "free_stand is run" << std::endl;

}

void Free_Stand_State::exit(){

}

FSMStateName Free_Stand_State::CheckChange(){
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
        if( user == UserValue::PASSIVE)
            return FSMStateName::PASSIVE;
        else if( user == UserValue::STAND)
            return FSMStateName::STAND;
        return FSMStateName::FREE_STAND;
}
Eigen::Matrix<float,3,4> Free_Stand_State::Cal_PosinB(float roll,float pitch,float yaw,float height){
   // printf("roll:%.3f pitch:%.3f yaw:%.3f h:%3f\n",roll,pitch,yaw,height);
    Eigen::Matrix<float,3,1> p1b = -_pb1_pos_b;
    p1b(2)+=height;
    Eigen::Matrix<float,3,3>rot = Rpy2RotMat(roll,pitch,yaw);

    Eigen::Matrix<float, 4, 4> Tsb = homoMatrix(p1b,rot);
    Eigen::Matrix<float, 4, 4> Tbs = homoMatrixInverse(Tsb);

    Eigen::Matrix<float, 4, 1> tempvec4;
    Eigen::Matrix<float, 3, 4> vecpb;

    for(int i(0); i<4; ++i){
        tempvec4 = Tbs * homoVec(_pos_s.col(i));
        vecpb.col(i) = noHomoVec(tempvec4);
    }
    // printf("x:%.3f y:%.3f z:%.3f \n",roll,pitch,yaw);

    return vecpb;
}

void Free_Stand_State::CalQ(Eigen::Matrix<float,3,4> pos){
    Eigen::Matrix<float,3,4> cmd_q;

    for(int i(0);i<4;++i){
        float length = _length_;
        float weigh = _weigh_;
        if(i == 0){
        weigh = -weigh;
        }
        else if(i == 3){
            length = -length;
        }
        else if(i == 2){
            weigh = -weigh;
            length = -length;
        }

        cmd_q.col(i) = Reversal_Solution_Update(i,pos.col(i)(0)-length,pos.col(i)(1)-weigh,pos.col(i)(2));

        // printf("id:%d 髋: %.3f , 大: %.3f , 小: %.3f \n",i,cmd_q.col(i)(0)/M_PI*180,cmd_q.col(i)(1)/M_PI*180,cmd_q.col(i)(2)/M_PI*180);
        // syslog(LOG_INFO,"id:%d 小: %.3f , 大: %.3f , 髋: %.3f \n",i,cmd_q.col(i)(0)/M_PI*180,cmd_q.col(i)(1)/M_PI*180,cmd_q.col(i)(2)/M_PI*180);
        _fstate_ctrl->_ioros->SetQ(i,cmd_q.col(i));

    }
    // Eigen::Matrix<float,3,1> position;

    // position = GetPos_H(0,_fsm_state_lowstate->Motor_Angle[0], _fsm_state_lowstate->Motor_Angle[1],_fsm_state_lowstate->Motor_Angle[2]);
    // syslog(LOG_INFO," x:%.3f -- y:%.3f -- z:%.3f ",position(0),position(1),position(2));
    // printf("x: %.3f , y: %.3f , z: %.3f \n",pos.col(0)(0)-length,pos.col(0)(1)-weigh,pos.col(0)(2));

}


Free_Stand_State::~Free_Stand_State(){
    
}
