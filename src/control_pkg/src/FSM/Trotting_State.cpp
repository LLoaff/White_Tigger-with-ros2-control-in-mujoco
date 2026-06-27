#include "FSM/Trotting_State.h"

Trotting_State::Trotting_State(ControlComponent * ctrlComp):FSMState(ctrlComp,FSMStateName::TROTTING,"trotting"),
_est(ctrlComp->_estimator),_phase(ctrlComp->_phase),_contact(ctrlComp->_contact),_lowstate(&ctrlComp->_ioros->_state)
,_balCtrl(ctrlComp->_balCtrl){
    _gait = new GaitGenerator(ctrlComp);
    _gaitHeight = 0.03;

    _vxLim << -0.5, 0.5;
    _vyLim << -0.5, 0.5;
    _wyawLim << -1.0, 1.0;
    _KPSwing<< 260.0, 0.0,   0.0,
                0.0, 300.0, 0.0,
                0.0, 0.0,   320.0;
    _KDSwing<< 8.0, 0.0, 0.0,
                0.0, 8.0, 0.0,
                0.0, 0.0, 10.0;
    

    _KPStance<< 260.0,  0.0,   0.0,
                0.0,  260.0,  0.0,
                0.0,  0.0,   520.0;

    _KDStance<< 8.0,  0.0,  0.0,
                 0.0,  8.0,  0.0,
                 0.0,  0.0,  14.0;

    _KPSwing_BACK  << 400.0, 0.0,   0.0,
                      0.0, 400.0, 0.0,
                      0.0, 0.0,   550.0;
    _KPStance_BACK << 400.0,  0.0,   0.0,
                     0.0,  400.0,  0.0,
                     0.0,  0.0,   750.0;
    // _KPSwing<< 260.0, 0.0,   0.0,
    //             0.0, 260.0, 0.0,
    //             0.0, 0.0,   320.0;
    // _KDSwing<< 8.0, 0.0, 0.0,
    //             0.0, 8.0, 0.0,
    //             0.0, 0.0, 10.0;
    

    // _KPStance<< 260.0,  0.0,   0.0,
    //             0.0,  260.0,  0.0,
    //             0.0,  0.0,   520.0;

    // _KDStance<< 8.0,  0.0,  0.0,
    //              0.0,  8.0,  0.0,
    //              0.0,  0.0,  14.0;

    // _KPSwing_BACK  << 300.0, 0.0,   0.0,
    //                 0.0, 300.0, 0.0,
    //                 0.0, 0.0,   430.0;
    // _KPStance_BACK << 320.0,  0.0,   0.0,
    //                  0.0,  320.0,  0.0,
    //                  0.0,  0.0,   600.0;

    _KP = _KPSwing;
    _KD = _KDSwing;

    // _KP_BACK = _KPSwing;
    // _KD_BACK = _KDSwing;
    // _Kpp = Vec3(70, 70, 70).asDiagonal();
    // _Kdp = Vec3(10, 10, 10).asDiagonal();
    // _kpw = 780;
    // _Kdw = Vec3(70, 70, 70).asDiagonal();
    // _KpSwing = Vec3(400, 400, 400).asDiagonal();
    // _KdSwing = Vec3(10, 10, 10).asDiagonal();
    _lcm = new lcm::LCM();
    _lcm2 = new lcm::LCM();
    _lcm3 = new lcm::LCM();
    _lcm4 = new lcm::LCM();
}

void Trotting_State::enter(){
    // _pcd = _est->getPcom();
    // _pcd(2) = 0.21;

    _vCmdBody.setZero();
    _wCmdGlobal.setZero();
    _dYawCmdPast = 0.0;

    // _yawCmd = _lowstate->_imu.getYaw();
    // _Rd = rotz(_yawCmd);

    _gait->restart();
    // _fstate_ctrl->waveGen->reset(
    //     0.5,
    //     0.75,
    //     Vec4(0.25, 0.75, 0.5, 0)
    // );
    _fstate_ctrl->waveGen->reset(
        0.7,        
        0.5,        
        Vec4(0, 0.5, 0.5, 0)  
    );
    // _fstate_ctrl->_is_wbc_run = true;
    std::cout<<"trotting"<<std::endl;
}

void Trotting_State::run(){
    // _posBody = _est->getPosition();
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;
    // std::cout<<"Roll: "<< _lowstate->_imu.getRoll() <<std::endl;
    // std::cout<<"Pitch: "<< _lowstate->_imu.getPitch() <<std::endl;
    // std::cout<<"Yaw: "<< _lowstate->_imu.getYaw() <<std::endl;
    // _velBody = _est->getVelocity();
    // _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    // _posFeetGlobal = _est->getFeetPos();
    // std::cout<<"_posFeetGlobal:\n"<< _posFeetGlobal <<std::endl;

    // _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowstate->_imu.GetRotMat().cast<double>();
    _G2B_RotMat = _B2G_RotMat.transpose();

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

    _gait->setGait(_vCmdBody.segment(0,2), _wCmdGlobal(2), _gaitHeight,_G2B_RotMat);
    _gait->run(_posFeet2BGoal, _velFeet2BGoal, _fstate_ctrl->_period, _fstate_ctrl->_stancePhaseRatio,FSMStateName::TROTTING); // 生成 body 坐标系下的足端目标
    // sendPlot((float)(*_contact)(0),0,0);

    // _feetCal->cal(_posFeetGlobalGoal,_velFeetGlobalGoal,_vCmdGlobal,_wCmdGlobal(2));
    // sendPlot((float)_posFeetGlobalGoal(0,0),(float)_posFeetGlobalGoal(1,0),(float)_posFeetGlobalGoal(2,0));
    //         //      全局 v                      全局角速度        抬腿高度
    // _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    // _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal, _fstate_ctrl->_period, _fstate_ctrl->_stancePhaseRatio,FSMStateName::TROTTING); // 生成下一刻的 足端坐标 、速度 (global)

    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl; // Z为正数 0.04
    // std::cout<<"_velFeetGlobalGoal:\n"<< _velFeetGlobalGoal <<std::endl;

    // std::cout<<"next_pos:\n"<< _posFeet2BGoal<<std::endl;
    this->calcTau();
    calcQQd(); // 计算机身坐标系下 足端坐标、速度

    // std::cout<<"_dq:\n"<< _qdGoal<< "\n"<<std::endl;
    const bool isStepping = checkStepOrNot();
    if(isStepping){
        _fstate_ctrl->setStartWave();
    }else{
        _fstate_ctrl->setAllStance();
    }

    // std::cout<<"_posFeet2BGoal:\n"<< _posFeet2BGoal <<std::endl;

    _fstate_ctrl->_ioros->SetQ(vec34ToVec12 (_qGoal.cast<float>()));
    _fstate_ctrl->_ioros->SetTau(_tau.cast<float>());
    _fstate_ctrl->_ioros->SetDq(vec34ToVec12 (_qdGoal.cast<float>()));

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
    _vCmdBody(1) = invNormalize(_userValue(1), _vyLim(0), _vyLim(1));// 换算y上期望速度
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue(3), _wyawLim(0), _wyawLim(1));// 换算转动期望速度
    _dYawCmd = 0.8*_dYawCmdPast + (1-0.8) * _dYawCmd;                 // 低通滤波

    _dYawCmdPast = _dYawCmd;

    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;   
    // std::cout<< "_dYawCmd: "<< _dYawCmd << std::endl;
}

void Trotting_State::calcCmd(){

    _vCmdGlobal = _B2G_RotMat *_vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_vxLim(0), _vxLim(1)));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_vyLim(0), _vyLim(1)));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _fstate_ctrl->dt, Vec2(_posBody(0) - 0.02, _posBody(0) + 0.02));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _fstate_ctrl->dt, Vec2(_posBody(1) - 0.02, _posBody(1) + 0.02));

    _vCmdGlobal(2) = 0;

    _yawCmd = _yawCmd + _dYawCmd * _fstate_ctrl->dt;
    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;

    // std::cout<< "_B2G_RotMat: \n"<< _B2G_RotMat << std::endl;
    // std::cout<< "_vCmdBody: \n"<< _vCmdBody << std::endl;
    // std::cout<< "_vCmdGlobal: \n"<< _vCmdGlobal << std::endl;
    // std::cout<< "_wCmdGlobal:\n"<<_wCmdGlobal<<std::endl;
    // std::cout<< "_pcd:\n"<<_pcd<<std::endl;
    // std::cout<< "_yawCmd:"<<_yawCmd<<std::endl;
}
void Trotting_State::calcTau(){
    // _posError = _pcd - _posBody;
    // _velError = _vCmdGlobal - _velBody;
    // std::cout<< "_pcd:\n"<<_pcd<<std::endl;
    // std::cout<< "_posBody:\n"<<_posBody<<std::endl;

    // std::cout<< "_posError:\n"<<_posError<<std::endl;
    // std::cout<< "_velError:"<<_velError<<std::endl;
    // _ddPcd = _Kpp * _posError + _Kdp * _velError;
    // _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowstate->_imu.getGyroGlobal().cast<double>());

    // _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    // _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    // _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    // _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    // _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    // _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    // _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    // for(int i(0); i<4; ++i){
    //     if((*_contact)(i) == 0){
    //         _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
    //     }
    // }
    // std::cout<<"_forceFeetGlobal\n"<<_forceFeetGlobal<<std::endl;

    // _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    // std::cout<<"_forceFeetBody\n"<<_forceFeetBody<<std::endl;
    // _q = vec34ToVec12(_fstate_ctrl->_ioros->getQ()).cast<double>();
    // _tau = getTau(_q.cast<float>(), _forceFeetBody);

    // std::cout<<"_tau\n"<<_tau<<std::endl;
}


void Trotting_State::calcQQd(){
    Vec34 _posFeet2B;
    _posFeet2B = GetFeetPos2BODY(*_lowstate,FrameType::BODY);
    // std::cout<<"_G2B_RotMat:\n"<< _G2B_RotMat <<std::endl;
    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;
    // std::cout<< "_pcd:\n"<<_pcd<<std::endl;
    // _posFeet2BGoal = _B2G_RotMat*_posFeet2BGoal;
    // std::cout<<"_posFeet2BGoal:\n"<< _posFeet2BGoal <<std::endl;

    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;
    sendPlot((float)_posFeet2BGoal(0,0),(float)_posFeet2BGoal(1,0),(float)_posFeet2BGoal(2,0));
    sendPlot2((float)_posFeet2BGoal(0,1),(float)_posFeet2BGoal(1,1),(float)_posFeet2BGoal(2,1));
    sendPlot3((float)_posFeet2BGoal(0,2),(float)_posFeet2BGoal(1,2),(float)_posFeet2BGoal(2,2));
    sendPlot4((float)_posFeet2BGoal(0,3),(float)_posFeet2BGoal(1,3),(float)_posFeet2BGoal(2,3));

    
    // Eigen::Matrix<double,3,4> pos = GetFeetPos2BODY(*_lowstate, FrameType::BODY);
    // sendPlot2((float)pos(0,0),(float)pos(1,0),(float)pos(2,0));
    // std::cout<<"pos:\n"<< pos <<std::endl;

    _qGoal = vec12ToVec34(Reversal_GetQ(_posFeet2BGoal, FrameType::BODY));
    // std::cout<<"_qGoal: \n"<<_qGoal<<std::endl;
    _qdGoal = vec12ToVec34(Reversal_GetQd(_posFeet2BGoal, _velFeet2BGoal, FrameType::BODY));
    _qqq = _fstate_ctrl->_ioros->getQ12();
    _www = _fstate_ctrl->_ioros->getW12();
    _tau.setZero();
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
        // const auto &kp = ((*_contact)(i) == 0) ? _KPSwing : _KPStance;
        // const auto &kd = ((*_contact)(i) == 0) ? _KDSwing : _KDStance;


        _tau.segment(3*i, 3) = CalTau(i,
                                      _qqq.segment(3*i, 3),
                                      _www.segment(3*i, 3),
                                      kp,
                                      kd,
                                      _posFeet2BGoal.col(i).cast<float>(),
                                      _velFeet2BGoal.col(i).cast<float>(),
                                      FrameType::BODY).cast<double>();
    }

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
    // delete _mpc;
    // delete _feetCal;
}
