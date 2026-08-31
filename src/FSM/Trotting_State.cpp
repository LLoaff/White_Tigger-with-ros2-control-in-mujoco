#include "FSM/Trotting_State.h"

Trotting_State::Trotting_State(ControlComponent * ctrlComp):FSMState(ctrlComp,FSMStateName::TROTTING,"trotting"),
_est(ctrlComp->_estimator),_phase(ctrlComp->_phase),_contact(ctrlComp->_contact),_lowstate(ctrlComp->_ioros->_state)
,_balCtrl(ctrlComp->_balCtrl),_robModel(ctrlComp->robotModel){
    _gait = new GaitGenerator(ctrlComp);
    if(use_go1_model == 1){
        _gaitHeight = 0.08;
        _vxLim << -0.8, 0.8;
        _vyLim << -0.8, 0.8;
        _wyawLim << -0.5, 0.5;

        _Kpp = Vec3(90, 90, 90).asDiagonal();
        _Kdp = Vec3(10, 10, 10).asDiagonal();
        _kpw = 780;
        _Kdw = Vec3(70, 70, 70).asDiagonal();

        _KpSwing = Vec3(400, 400, 400).asDiagonal();
        _KdSwing = Vec3(10, 10, 10).asDiagonal();
    }else{
        _gaitHeight = 0.04;
        _vxLim << -0.4, 0.4;
        _vyLim << -0.4, 0.4;
        _wyawLim << -0.5, 0.5;

        // _Kpp = Vec3(15, 15, 15).asDiagonal();
        // _Kdp = Vec3(1.5, 1.5, 1.5).asDiagonal();
        // _kpw = 120;
        // _Kdw = Vec3(15, 15, 15).asDiagonal();

        // _KpSwing = Vec3(25, 25, 25).asDiagonal();
        // _KdSwing = Vec3(0.6, 0.6, 0.6).asDiagonal();
        _Kpp = Vec3(45, 45, 45).asDiagonal();
        _Kdp = Vec3(5, 5, 5).asDiagonal();
        _kpw = 230;
        _Kdw = Vec3(22, 22, 22).asDiagonal();

        _KpSwing = Vec3(90, 90, 90).asDiagonal();
        _KdSwing = Vec3(2, 2, 2).asDiagonal();
    }
    
    // 向网端发送param数据
    _fstate_ctrl->_analyze._param._Kpp = _Kpp;
    _fstate_ctrl->_analyze._param._Kdp = _Kdp;
    _fstate_ctrl->_analyze._param._kpw = _kpw;
    _fstate_ctrl->_analyze._param._Kdw = _Kdw;
    _fstate_ctrl->_analyze._param._KpSwing = _KpSwing;
    _fstate_ctrl->_analyze._param._KdSwing = _KdSwing;
    _fstate_ctrl->_analyze.sendParamData(_fstate_ctrl->_analyze._param);

    _lcm = new lcm::LCM();
    _lcm2 = new lcm::LCM();
    _lcm3 = new lcm::LCM();
    _lcm4 = new lcm::LCM();
}

void Trotting_State::enter(){
    _pcd = _est->getPosition();
    if(use_go1_model == 1){
        _pcd(2) = 0.32;
    }else{
        _pcd(2) = 0.2;
    }

    _vCmdBody.setZero();
    _wCmdGlobal.setZero();
    _dYawCmdPast = 0.0;

    _yawCmd = _lowstate->_imu.getYaw();
    _Rd = rotz(_yawCmd);


    _fstate_ctrl->waveGen->reset(
        0.5,        
        0.5,        
        Vec4(0, 0.5, 0.5, 0),
        _fstate_ctrl->_mjdata->time
    );
    _gait->restart();
    if (_fstate_ctrl->_analyze.getParamData(_fstate_ctrl->_analyze._param)){
        _Kpp = _fstate_ctrl->_analyze._param._Kpp;
        _Kdp = _fstate_ctrl->_analyze._param._Kdp;
        _kpw = _fstate_ctrl->_analyze._param._kpw;
        _Kdw = _fstate_ctrl->_analyze._param._Kdw;

        _KpSwing = _fstate_ctrl->_analyze._param._KpSwing;
        _KdSwing = _fstate_ctrl->_analyze._param._KdSwing;
        std::cout<<"参数读取成功"<<std::endl;
        std::cout<<"当前 _Kpp:\n"
        << _Kpp 
        <<"\n_Kdp:\n"<<_Kdp
        <<"\n_kpw:\n"<<_kpw
        <<"\n_Kdw:\n"<<_Kdw
        <<"\n_KpSwing:\n"<<_KpSwing
        <<"\n_KdSwing:\n"<<_KdSwing
        <<std::endl;
    }
    

    std::cout<<"trotting"<<std::endl;
}

void Trotting_State::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowstate->_imu.GetRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowstate->_imu.getYaw();
    _dYaw = _lowstate->_imu.getDYaw();
   /* 遥控 */
    _userValue(0) = _fstate_ctrl->user_cmd->_vx ;
    _userValue(1) = _fstate_ctrl->user_cmd->_vy;
    _userValue(2) = 0;
    _userValue(3) = _fstate_ctrl->user_cmd->_wz;

    getUserCmd(); // 计算 期望速度
    calcCmd();    // 计算位移、转动角度，获取全局速度

    _gait->setGait(_vCmdBody.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal, _fstate_ctrl->_period, _fstate_ctrl->_stancePhaseRatio,FSMStateName::TROTTING); // 生成 body 坐标系下的足端目标
    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl; // Z为正数 0.04
    // std::cout<<"_velFeetGlobalGoal:\n"<< _velFeetGlobalGoal <<std::endl;
    // std::cout<<"next_pos:\n"<< _posFeet2BGoal<<std::endl;
    // std::cout<<"_yaw:\n"<< _yaw<<std::endl;
    calcTau();
    calcQQd(); // 计算机身坐标系下 足端坐标、速度

    // std::cout<<"_dq:\n"<< _qdGoal<< "\n"<<std::endl;
    const bool isStepping = checkStepOrNot();

    if(isStepping){
        _fstate_ctrl->setStartWave();
    }else{
        _fstate_ctrl->setAllStance();
    }

    // std::cout<<"_posFeet2BGoal:\n"<< _posFeet2BGoal <<std::endl;

    _fstate_ctrl->_ioros->SetTau(_tau);
    _fstate_ctrl->_ioros->SetQ(vec34ToVec12 (_qGoal));
    _fstate_ctrl->_ioros->SetDq(vec34ToVec12 (_qdGoal));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _fstate_ctrl->_ioros->setSwingGain(i);
            // _fstate_ctrl->_ioros->setZeroGain(i);
        }else{
            _fstate_ctrl->_ioros->setStableGain(i);
            // _fstate_ctrl->_ioros->setZeroGain(i);
        }
    }
}

bool Trotting_State::checkStepOrNot(){

    // static bool isStepping = false;

    // if(isStepping){
    //     if( (fabs(_vCmdBody(0)) < 0.02) &&
    //         (fabs(_vCmdBody(1)) < 0.02) &&
    //         (fabs(_dYawCmd) < 0.03) ){
    //         isStepping = false;
    //     }
    // }else{
    //     if( (fabs(_vCmdBody(0)) > 0.05) ||
    //         (fabs(_vCmdBody(1)) > 0.05) ||
    //         (fabs(_dYawCmd) > 0.08) ){
    //         isStepping = true;
    //     }
    // }

    // return isStepping;
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

void Trotting_State::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue(0), _vxLim(0), _vxLim(1));// 换算x上期望速度
    _vCmdBody(1) = invNormalize(_userValue(1), _vyLim(0), _vyLim(1));// 换算y上期望速度
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue(3), _wyawLim(0), _wyawLim(1));// 换算转动期望速度
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;                 // 低通滤波
    
    _dYawCmdPast = _dYawCmd;

}

void Trotting_State::calcCmd(){

    _vCmdGlobal = _B2G_RotMat *_vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_vxLim(0)-0.2, _vxLim(1)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_vyLim(0)-0.2, _vyLim(1)+0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _fstate_ctrl->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _fstate_ctrl->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    _yawCmd = _yawCmd + _dYawCmd * _fstate_ctrl->dt;
    if(_yawCmd > M_PI) _yawCmd -=2*M_PI;
    else if(_yawCmd < -M_PI) _yawCmd +=2*M_PI;
    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

void Trotting_State::calcTau(){
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowstate->_imu.getGyroGlobal());
    // _dWbd  =  _Kdw * (_wCmdGlobal - _lowstate->_imu.getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-60, 60));
    Vec3 werror;
    _fstate_ctrl->_analyze.sendComPos(_fstate_ctrl->_mjdata->time,_posError,_dWbd);
    // std::cout<<"_dWbd:\n"<< _dWbd <<std::endl;
    // std::cout
    // << " yawCmd=" << _yawCmd
    // << " dYawCmd=" << _dYawCmd
    // << " _wCmdGlobal=" << _wCmdGlobal(2)
    // << " getGyroGlobal=" << _lowstate->_imu.getGyroGlobal()(2)
    // << " _dWbd=" << _dWbd(2)
    // << " _Rd = \n" << _Rd
    // << std::endl;
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }
    // std::cout<<"_forceFeetGlobal\n"<<_forceFeetGlobal<<std::endl;

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_fstate_ctrl->_ioros->getQ());
    _tau = getTau(_q, _forceFeetBody);
    // std::cout<<"_pcd\n"<<_pcd<<std::endl;

    // std::cout<<"_tau\n"<<_tau<<std::endl;
    // std::cout<<"_forceFeetBody\n"<<_forceFeetBody<<std::endl;
}

void Trotting_State::calcQQd(){
    Vec34 _posFeet2B;
    _posFeet2B = GetFeetPos2BODY(*_lowstate,FrameType::BODY);
    for (int i(0); i < 4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
    }
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;
    // std::cout<<"_posFeet2BGoal:\n"<< _posFeet2BGoal <<std::endl;

    // std::cout<<"_posFeetGlobalGoal:\n"<< _posFeetGlobalGoal <<std::endl;
    // std::cout<<"_posBody:\n"<< _posBody <<std::endl;
    // Vec34 endpos = _gait->getEndpos();

    // sendPlot((float)_posFeet2BGoal(0,0),(float)_posFeet2BGoal(1,0),(float)_posFeet2BGoal(2,0));
    // sendPlot2((float)_posFeet2BGoal(0,1),(float)_posFeet2BGoal(1,1),(float)_posFeet2BGoal(2,1));
    // sendPlot3((float)_posFeet2BGoal(0,2),(float)_posFeet2BGoal(1,2),(float)_posFeet2BGoal(2,2));
    // sendPlot4((float)_posFeet2BGoal(0,3),(float)_posFeet2BGoal(1,3),(float)_posFeet2BGoal(2,3));
    // sendPlot((float)endpos(0,0),(float)endpos(1,0),(float)endpos(2,0));
    // sendPlot2((float)endpos(0,1),(float)endpos(1,1),(float)endpos(2,1));
    // sendPlot3((float)endpos(0,2),(float)endpos(1,2),(float)endpos(2,2));
    // sendPlot4((float)endpos(0,3),(float)endpos(1,3),(float)endpos(2,3));

    _qGoal = vec12ToVec34(Reversal_GetQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(Reversal_GetQd(_posFeet2BGoal, _velFeet2BGoal, FrameType::BODY));

    // std::cout<<"_qGoal: \n"<<_qGoal<<std::endl;
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
    else if ( user == UserValue::BALANCE)
        return FSMStateName::BALANCE;
    return FSMStateName::TROTTING;
}

Trotting_State::~Trotting_State(){
    // delete _gait;
    // delete _mpc;
    // delete _feetCal;
}
