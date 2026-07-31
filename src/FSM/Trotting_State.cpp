#include "FSM/Trotting_State.h"

Trotting_State::Trotting_State(ControlComponent * ctrlComp):FSMState(ctrlComp,FSMStateName::TROTTING,"trotting"),_phase(ctrlComp->_phase),_contact(ctrlComp->_contact),_lowstate(&ctrlComp->_ioros->_state){
    _gait = new GaitGenerator(ctrlComp);
    _gaitHeight = 0.04;

    _vxLim << -0.8, 0.8;
    _vyLim << -0.8, 0.8;
    _wyawLim << -1.5, 1.5;
    _KPSwing<< 100.0, 0.0,   0.0,
                0.0, 100.0, 0.0,
                0.0, 0.0,   180.0;
    _KDSwing<< 20.0, 0.0, 0.0,
                0.0, 20.0, 0.0,
                0.0, 0.0, 15.0;

    // 前腿支撑相
    _KPStance<< 100.0,  0.0,   0.0,
                0.0,  100.0,  0.0,
                0.0,  0.0,   180.0;
    _KDStance<< 15.0,  0.0,  0.0,
                0.0,  15.0,  0.0,
                0.0,  0.0,  15.0;

    // 后腿摆动相
    _KPSwing_BACK  << 100.0, 0.0,   0.0,
                    0.0, 100.0, 0.0,
                    0.0, 0.0,   150.0;

    // 后腿支撑相
    _KPStance_BACK << 100.0,  0.0,   0.0,
                    0.0,  100.0,  0.0,
                    0.0,  0.0,   150.0;

    _KP = _KPSwing;
    _KD = _KDSwing;
    _lcm = new lcm::LCM();
    _lcm2 = new lcm::LCM();
    _lcm3 = new lcm::LCM();
    _lcm4 = new lcm::LCM();
}

void Trotting_State::enter(){
 
    // _pcd = _est->getPosition() ;
    // _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    
    _vCmdBody.setZero();
    _wCmdGlobal.setZero();
    _dYawCmdPast = 0.0;

    _gait->restart();
    _fstate_ctrl->waveGen->reset(
        0.5,        
        0.5,        
        Vec4(0, 0.5, 0.5, 0)  
    );
    std::cout<<"trotting"<<std::endl;
}

void Trotting_State::run(){
    _userValue(0) = _fstate_ctrl->user_cmd->_vx ;
    _userValue(1) = _fstate_ctrl->user_cmd->_vy;
    _userValue(2) = 0;
    _userValue(3) = _fstate_ctrl->user_cmd->_wz;
    getUserCmd(); // 计算 期望速度
    calcCmd();    // 计算位移、转动角度，获取全局速度

    _gait->setGait(_vCmdBody.segment(0,2), _wCmdGlobal(2), _gaitHeight,_G2B_RotMat);
    _gait->run(_posFeet2BGoal, _velFeet2BGoal, _fstate_ctrl->_period, _fstate_ctrl->_stancePhaseRatio,FSMStateName::TROTTING); // 生成 body 坐标系下的足端目标

    calcQQd(); // 计算机身坐标系下 足端坐标、速度

    // std::cout<<"_dq:\n"<< _qdGoal<< "\n"<<std::endl;
    bool isStepping = checkStepOrNot();
    if(isStepping){
        _fstate_ctrl->setStartWave();
    }else{
        _fstate_ctrl->setAllStance();
    }


    _fstate_ctrl->_ioros->SetQ(vec34ToVec12 (_qGoal.cast<float>()));
    // _fstate_ctrl->_ioros->SetTau(_tau.cast<float>());
    // _fstate_ctrl->_ioros->SetDq(vec34ToVec12 (_qdGoal.cast<float>()));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _fstate_ctrl->_ioros->setSwingGain(i);
        }else{
            _fstate_ctrl->_ioros->setStableGain(i);
        }
    }
}

bool Trotting_State::checkStepOrNot(){

    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_dYawCmd) > 0.05) ){
        return true;
    }else{
        return false;
    }

    // static bool isStepping = false;
    
    // if(isStepping){
    //     // 已经在迈步，只有当速度低于下阈值时才停止
    //     if( (fabs(_vCmdBody(0)) < 0.02) &&
    //         (fabs(_vCmdBody(1)) < 0.02) &&
    //         (fabs(_dYawCmd) < 0.03) ){
    //         isStepping = false;
    //     }
    // }else{
    //     // 没有迈步，只有当速度高于上阈值时才开始
    //     if( (fabs(_vCmdBody(0)) > 0.05) ||
    //         (fabs(_vCmdBody(1)) > 0.05) ||
    //         (fabs(_dYawCmd) > 0.08) ){
    //         isStepping = true;
    //     }
    // }
    
    // return isStepping;
}

// 期望速度 _vCmdBody：OK  全局期望速度 _vCmdGlobal：OK  全局期望角速度_wCmdGlobal：OK
// 质心位移 _pcd：OK
void Trotting_State::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue(0), _vxLim(0), _vxLim(1));// 换算x上期望速度
    _vCmdBody(1) = invNormalize(_userValue(1), _vyLim(0), _vyLim(1));// 换算y上期望速度
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue(3), _wyawLim(0), _wyawLim(1));// 换算转动期望速度
    _dYawCmd = 0.8*_dYawCmdPast + (1-0.8) * _dYawCmd;                 // 低通滤波

    _dYawCmdPast = _dYawCmd;

}

void Trotting_State::calcCmd(){

    //  _vCmdGlobal = _B2G_RotMat *_vCmdBody;
     _vCmdGlobal = _vCmdBody;


    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_vxLim(0), _vxLim(1)));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_vyLim(0), _vyLim(1)));

    _vCmdGlobal(2) = 0;

    _yawCmd = _yawCmd + _dYawCmd * _fstate_ctrl->dt;
    // _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;


}

void Trotting_State::calcTau(){
}

void Trotting_State::calcQQd(){

    sendPlot((float)_posFeet2BGoal(0,0),(float)_posFeet2BGoal(1,0),(float)_posFeet2BGoal(2,0));
    sendPlot2((float)_posFeet2BGoal(0,1),(float)_posFeet2BGoal(1,1),(float)_posFeet2BGoal(2,1));
    sendPlot3((float)_posFeet2BGoal(0,2),(float)_posFeet2BGoal(1,2),(float)_posFeet2BGoal(2,2));
    sendPlot4((float)_posFeet2BGoal(0,3),(float)_posFeet2BGoal(1,3),(float)_posFeet2BGoal(2,3));
    _qGoal = vec12ToVec34(Reversal_GetQ(_posFeet2BGoal, FrameType::BODY));
    // std::cout<<"_qGoal: \n"<<_qGoal<<std::endl;
    _qdGoal = vec12ToVec34(Reversal_GetQd(_posFeet2BGoal, _velFeet2BGoal, FrameType::BODY));
    _qqq = _fstate_ctrl->_ioros->getQ12();
    _www = _fstate_ctrl->_ioros->getW12();
    // _tau.setZero();
    Eigen::Matrix<float,3,3> kp;
    Eigen::Matrix<float,3,3> kd;
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){
            if(i<2)
                kp = _KPStance;
            else kp = _KPStance_BACK;
            kd = _KDStance;
        }
        else{
            if(i<2) kp = _KPSwing;
            else kp = _KPSwing_BACK;
            kd = _KDSwing;

        }
        _tau.segment(3*i, 3) = CalTau(i,
                                      _qqq.segment(3*i, 3),
                                      _www.segment(3*i, 3),
                                      kp,
                                      kd,
                                      _posFeet2BGoal.col(i).cast<float>(),
                                      _velFeet2BGoal.col(i).cast<float>(),
                                      FrameType::BODY).cast<double>();
    }


}

void Trotting_State::exit(){
    _fstate_ctrl->setAllSwing();
}

FSMStateName Trotting_State::CheckChange(){
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
    if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;
    else if ( user == UserValue::STAND)
        return FSMStateName::STAND;
    else if ( user == UserValue::SIT_DOWN)
        return FSMStateName::SIT_DOWN;
    else if ( user == UserValue::JUMP)
        return FSMStateName::JUMP;
    return FSMStateName::TROTTING;
}

Trotting_State::~Trotting_State(){
    delete _gait;
}

void Trotting_State::sendPlot(float x,float y,float z){
    _msg.x = x;
    _msg.y = y;
    _msg.z = z;
    _lcm->publish("plot_rf",&_msg);
}

void Trotting_State::sendPlot2(float x,float y,float z){
    _msg.x = x;
    _msg.y = y;
    _msg.z = z;
    _lcm->publish("plot_lf",&_msg);
}
void Trotting_State::sendPlot3(float x,float y,float z){
    _msg.x = x;
    _msg.y = y;
    _msg.z = z;
    _lcm->publish("plot_rr",&_msg);
}
void Trotting_State::sendPlot4(float x,float y,float z){
    _msg.x = x;
    _msg.y = y;
    _msg.z = z;
    _lcm->publish("plot_lr",&_msg);
}