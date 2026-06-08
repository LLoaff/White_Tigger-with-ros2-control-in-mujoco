#include "FSM/Balance_State.h"


Balance_State::Balance_State(ControlComponent * balance_ctrl_comp,LowState* lowstate):FSMState(balance_ctrl_comp,FSMStateName::BALANCE,"balance")
,_est(balance_ctrl_comp->_estimator),_balCtrl(balance_ctrl_comp->balCtrl),_contact(balance_ctrl_comp->_contact),_lowstate(lowstate){

    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    _Kpp = Vec3(400, 400, 500).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();

    _kpw = 400;
    _Kdw = Vec3(10, 10, 10).asDiagonal();
}

void Balance_State::enter(){
    Eigen::Matrix<float,3,1> kp,kd;
    // kp<< 100 , 100 , 120;
    // kd<< 7 , 7 , 7;
     kp<< 2 , 2 , 3.2;
    kd<< 0.7 , 0.7 , 0.7;
    for(int i=0;i<4;i++)
    {
        _fstate_ctrl->_ioros->SetP(i,kp);
        _fstate_ctrl->_ioros->SetD(i,kd);
    }

    _pcdInit = _est->getPosition();
    _pcd = _pcdInit;
    _RdInit = _lowstate->_imu.GetRotMat().cast<double>();
    _fstate_ctrl->setAllStance();
}

void Balance_State::run(){
    /* 遥控归一 */

    float ly = (_fstate_ctrl->user_cmd->R_Data.ch3 - 1024) / 660.0;
    float lx = (_fstate_ctrl->user_cmd->R_Data.ch2 - 1024) / 660.0;
    float ry = (_fstate_ctrl->user_cmd->R_Data.ch1 - 1024) / 660.0;
    float rx = (_fstate_ctrl->user_cmd->R_Data.ch0 - 1024) / 660.0;

    _pcd(0) = _pcdInit(0) + invNormalize(ly, _xMin, _xMax);
    _pcd(1) = _pcdInit(1) - invNormalize(lx, _yMin, _yMax);
    _pcd(2) = _pcdInit(2) + invNormalize(ry, _zMin, _zMax);

    float yaw = invNormalize(rx, _yawMin, _yawMax); 

    _Rd = Rpy2RotMat(0, 0, yaw).cast<double>()*_RdInit;

    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();

    _B2G_RotMat = _lowstate->_imu.GetRotMat().cast<double>();
    _G2B_RotMat = _B2G_RotMat.transpose();

    calcTau();

    // for(int i(0);i<12;++i){
    //     printf("id: %d  q: %.3f  tau: %.3f  \n",i,_q(i),_tau(i));
    // }
    // for(int i(0);i<4;++i){
    //     printf("id: %d  q: %.3f  tau: %.3f  \n",i,_q(i*3+2),_tau(i*3+2));
    // }

    // printf("x:%.3f y:%.3f z:%.3f h: %.3f \n",_pcd(0),_pcd(1),_pcd(2),_posBody(2));
    _fstate_ctrl->_ioros->SetTau(_tau.cast<float>());
    _fstate_ctrl->_ioros->SetQ(_q);
}

void Balance_State::exit(){

}

FSMStateName Balance_State::CheckChange(){
    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
    if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;
    else if( user == UserValue::STAND)
        return FSMStateName::STAND;
    return FSMStateName::BALANCE;
}

void Balance_State::calcTau(){
     _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Eigen::Matrix<double,3,1>(0, 0, 0) - _velBody);

     _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Eigen::Matrix<double,3,1>(0, 0, 0) - _lowstate->_imu.getGyroGlobal().cast<double>());

     _posFeet2BGlobal = _est->getPosFeet2BGlobal();
//
     Eigen::Matrix<double,3,4> _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

     _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    //  std::cout << "_G2B_RotMat: \n" << _G2B_RotMat << std::endl;
    // std::cout << " _forceFeetGlobal: \n" << _forceFeetGlobal << " ===" << std::endl;
    // std::cout << " _forceFeetBody: \n" << vec34ToVec12(_forceFeetBody.cast<float>()) << " \n===" << std::endl;

    _q = vec34ToVec12(_fstate_ctrl->_ioros->getQ());
    // std::cout << " _q: \n" << _q << " \n===" << std::endl;

    _tau = getTau(_q, _forceFeetBody);
    // std::cout << " _tau: \n" << _tau << " \n===" << std::endl;
}

Balance_State::~Balance_State(){

}
