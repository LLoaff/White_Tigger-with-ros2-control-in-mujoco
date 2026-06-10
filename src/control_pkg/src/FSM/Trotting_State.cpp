#include "FSM/Trotting_State.h"

Trotting_State::Trotting_State(ControlComponent * ctrlComp):FSMState(ctrlComp,FSMStateName::TROTTING,"trotting"),
_est(ctrlComp->_estimator),_phase(ctrlComp->_phase),_contact(ctrlComp->_contact),_lowstate(&ctrlComp->_ioros->_state)
,_feetCal(ctrlComp->_feetendcal){    
    // _feetCal = new FeetEndCal(ctrlComp);
    _vxLim << -0.5, 0.5;
    _vyLim << -0.5, 0.5;
    _wyawLim << -1.0, 1.0;

    _KP<< 10.0,  0 ,   0,
          0 ,   10.0,  0,
          0 ,   0 ,   10.0;

    _KD<< 3.0, 0,    0,
          0,   3.0,  0,
          0,   0,   3.0;
    _lcm = new lcm::LCM();
    if(!_lcm->good())        {
        std::cerr<<"LCM init failed!"<<std::endl;
    }
}

void Trotting_State::enter(){
    _vCmdBody.setZero();
    _wCmdGlobal.setZero();

    _fstate_ctrl->waveGen->reset(
        0.5,        
        0.5,        
        Vec4(0, 0.5, 0.5, 0)  
    );
    _fstate_ctrl->_is_wbc_run = true;
    std::cout<<"trotting"<<std::endl;
}

void Trotting_State::run(){
    // _G2B_RotMat = rotx(_lowstate->_imu.getRoll()).cast<double>()*roty(_lowstate->_imu.getPitch()).cast<double>();
//     std::cout<<"q0: "<<_fstate_ctrl->_ioros->_state._imu.quaternion[0]<<" q1: "<<_fstate_ctrl->_ioros->_state._imu.quaternion[1]<<
//    " q2: "<<_fstate_ctrl->_ioros->_state._imu.quaternion[2]<<" q3: "<<_fstate_ctrl->_ioros->_state._imu.quaternion[3]<<std::endl;
    // std::cout<<"Roll: "<< _lowstate->_imu.getRoll() <<std::endl;
    // std::cout<<"Pitch: "<< _lowstate->_imu.getPitch() <<std::endl;
    // std::cout<<"rotx: \n"<< rotx(_lowstate->_imu.getRoll()) <<std::endl;
    // std::cout<<"roty: \n"<< roty(_lowstate->_imu.getPitch()) <<std::endl;

    _yaw = _lowstate->_imu.getYaw() ;
    _dYaw = _lowstate->_imu.getDYaw();

// /* 遥控归一 */

    _userValue(0) = _fstate_ctrl->user_cmd->_vx ;
    _userValue(1) = _fstate_ctrl->user_cmd->_vy;
    _userValue(2) = 0;
    _userValue(3) = _fstate_ctrl->user_cmd->_wz;
    // std::cout<<"_userValue: "<< _userValue.transpose() <<std::endl;
    // _userValue(0) = _fstate_ctrl->_ioros->_cmd_vel[0];
    // _userValue(1) = _fstate_ctrl->_ioros->_cmd_vel[1];
    // _userValue(3) = _fstate_ctrl->_ioros->_cmd_vel[5];
    
    getUserCmd(); // 计算 期望速度
    calcCmd();    // 计算位移、转动角度，获取全局速度
    _feetCal->cal(_posFeetGlobalGoal,_velFeetGlobalGoal,_vCmdGlobal,_wCmdGlobal(2));
    // _mpc->update();
    // sendPlot((float)_posFeetGlobalGoal(0,0),(float)_posFeetGlobalGoal(1,0),(float)_posFeetGlobalGoal(2,0));
    //         //      全局 v                      全局角速度        抬腿高度
    // _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    // _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal, _fstate_ctrl->_period, _fstate_ctrl->_stancePhaseRatio,FSMStateName::TROTTING); // 生成下一刻的 足端坐标 、速度 (global)
    
    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_velFeetGlobalGoal:\n"<< _velFeetGlobalGoal <<std::endl;

    // std::cout<<"next_pos:\n"<< _posFeet2BGoal<<std::endl;
    // calcTau();
    // calcQQd(); // 计算机身坐标系下 足端坐标、速度
    
    // std::cout<<"_dq:\n"<< _qdGoal<< "\n"<<std::endl;
    if(checkStepOrNot()){
        _fstate_ctrl->setStartWave();
    }else{
        _fstate_ctrl->setAllStance();
    }
    
    // _fstate_ctrl->_ioros->SetQ(vec34ToVec12 (_qGoal.cast<float>()));
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
    // if( (fabs(_vCmdBody(0)) > 0.03) ||
    //     (fabs(_vCmdBody(1)) > 0.03) ||
    //     (fabs(_posError(0)) > 0.08) ||
    //     (fabs(_posError(1)) > 0.08) ||
    //     (fabs(_velError(0)) > 0.08) ||
    //     (fabs(_velError(1)) > 0.08) ||
    //     (fabs(_dYawCmd) > 0.05) ){
    //     return true;
    // }else{
    //     return false;
    // }

    // if( (fabs(_vCmdBody(0)) > 0.03) ||
    //     (fabs(_vCmdBody(1)) > 0.03) ||
    //     (fabs(_dYawCmd) > 0.05) ){
    //     return true;
    // }else{
    //     return false;
    // }

    static bool isStepping = false;
    
    if(isStepping){
        if( (fabs(_vCmdBody(0)) < 0.02) &&
            (fabs(_vCmdBody(1)) < 0.02) &&
            (fabs(_dYawCmd) < 0.03) ){
            isStepping = false;
        }
    }else{
        // 没有迈步，只有当速度高于上阈值时才开始
        if( (fabs(_vCmdBody(0)) > 0.05) ||
            (fabs(_vCmdBody(1)) > 0.05) ||
            (fabs(_dYawCmd) > 0.08) ){
            isStepping = true;
        }
    }
    
    return isStepping;
}


// 期望速度 _vCmdBody：OK  全局期望速度 _vCmdGlobal：OK  全局期望角速度_wCmdGlobal：OK
// 质心位移 _pcd：OK
void Trotting_State::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue(0), _vxLim(0), _vxLim(1));// 换算x上期望速度
    _vCmdBody(1) = -invNormalize(_userValue(1), _vyLim(0), _vyLim(1));// 换算y上期望速度
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue(3), _wyawLim(0), _wyawLim(1));// 换算转动期望速度
    _dYawCmd = 0.8*_dYawCmdPast + (1-0.8) * _dYawCmd;                 // 低通滤波

    _vxCmdPast=_vCmdBody(0);
    _vyCmdPast=_vCmdBody(1);
    _dYawCmdPast = _dYawCmd;

    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;   
    // std::cout<< "_dYawCmd: "<< _dYawCmd << std::endl;
}

void Trotting_State::calcCmd(){

    _vCmdGlobal = _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(-0.5, 0.5));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(-0.5, 0.5));

    _vCmdGlobal(2) = 0;
    _wCmdGlobal(2) = _dYawCmd;

    // std::cout<< "_B2G_RotMat: \n"<< _B2G_RotMat << std::endl;
    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;
    // std::cout<< "_vCmdGlobal: \n"<< _vCmdGlobal << std::endl;
    // std::cout<< "_wCmdGlobal:\n"<<_wCmdGlobal<<std::endl;
    // std::cout<< "_pcd:\n"<<_pcd<<std::endl;
    // std::cout<< "_yawCmd:"<<_yawCmd<<std::endl;
}


void Trotting_State::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = GetFeetPos2BODY(*_lowstate,FrameType::BODY);
    
    // for(int i(0); i<4; ++i){
    //     _posFeet2BGoal.col(i) = _posFeetGlobalGoal.col(i);
    //     _velFeet2BGoal.col(i) =  _velFeetGlobalGoal.col(i); 
    //     // _posFeet2BGoal.col(i) = (_posFeetGlobalGoal.col(i)- _posBody);
    //     // _velFeet2BGoal.col(i) =  (_velFeetGlobalGoal.col(i)- _velBody); 
    //     // std::cout<<"_posFeetGlobalGoal.col(i)- _posBody:\n"<< _posFeetGlobalGoal.col(i)- _posBody <<std::endl;
    //     // std::cout<<"_G2B_RotMat:\n"<< _G2B_RotMat <<std::endl;
    // }
    // std::cout<<"_G2B_RotMat:\n"<< _G2B_RotMat <<std::endl;

    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;

    // std::cout<<"_posFeet2BGoal:\n"<< _posFeet2BGoal <<std::endl;
    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;

    // usleep(50000);
    // sleep(1);
    // _qGoal = vec12ToVec34(Reversal_GetQ(_posFeet2BGoal, FrameType::BODY));
    // // std::cout<<"_q: \n"<<_qGoal<<std::endl;
    // _qdGoal = vec12ToVec34(Reversal_GetQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
    // _qqq = _fstate_ctrl->_ioros->getQ12();
    // _www = _fstate_ctrl->_ioros->getW12();
    // _tau = CalTaus(_qqq,_www,_KP,_KD,vec34ToVec12(_posFeet2BGoal.cast<float>()),vec34ToVec12(_velFeet2BGoal.cast<float>()),FrameType::BODY).cast<double>();

    // _tau.setZero();
    // for(int i = 0; i < 4; ++i) {
    //     _tau(i * 3 + 2) = 1.0; // 膝关节重力补偿
    // }
    // std::cout<<"_tau:\n"<< _tau <<std::endl;
    // std::cout<<"_qdGoal:\n"<< _qdGoal <<std::endl;

}

void Trotting_State::sendPlot(float x,float y,float z){
    _msg.x = x;
    _msg.y = y;
    _msg.z = z;
    
    _lcm->publish("plot",&_msg);
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
    return FSMStateName::TROTTING;
}

Trotting_State::~Trotting_State(){
    // delete _gait;
    delete _mpc;
    // delete _feetCal;
}