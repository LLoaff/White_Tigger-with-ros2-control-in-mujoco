/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_BalanceTest.h"

State_BalanceTest::State_BalanceTest(ControlComponent *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::BALANCE, "balanceTest"),
                  _est(ctrlComp->_estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->_balCtrl), _contact(ctrlComp->_contact){

    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    _Kpp = Vec3(500, 500, 500).asDiagonal();
    _Kdp = Vec3(25, 25, 25).asDiagonal();

    _kpw = 200;
    _Kdw = Vec3(30, 30, 30).asDiagonal();
}

void State_BalanceTest::enter(){
    _pcdInit = _est->getPosition();
    _pcd = _pcdInit;
    _RdInit = _fstate_ctrl->_ioros->_state._imu.GetRotMat();
    std::cout<<"balabce_test"<<std::endl;

    _fstate_ctrl->setAllStance();
}

void State_BalanceTest::run(){

    _userValue(0) = _fstate_ctrl->user_cmd->_vx ;// 实际是质心偏移位移
    _userValue(1) = _fstate_ctrl->user_cmd->_vy;
    _userValue(2) = 0;
    _userValue(3) = _fstate_ctrl->user_cmd->_wz;

    _pcd(0) = _pcdInit(0) + invNormalize(_userValue(1), _xMin, _xMax);
    _pcd(1) = _pcdInit(1) - invNormalize(_userValue(0), _yMin, _yMax);
    _pcd(2) = _pcdInit(2) + invNormalize(_userValue(3), _zMin, _zMax);

    float yaw = invNormalize(_userValue(2), _yawMin, _yawMax);
    _Rd = Rpy2RotMat(0, 0, yaw)*_RdInit;

    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();

    _B2G_RotMat = _fstate_ctrl->_ioros->_state._imu.GetRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    calcTau();
    _fstate_ctrl->_ioros->setStableGain();
    _fstate_ctrl->_ioros->SetTau(_tau);
    _fstate_ctrl->_ioros->SetQ(_q);

}

void State_BalanceTest::exit(){
}

FSMStateName State_BalanceTest::CheckChange(){

    UserValue user = _fstate_ctrl->user_cmd->GetUserValue();
    if( user == UserValue::PASSIVE)
        return FSMStateName::PASSIVE;
    else if ( user == UserValue::STAND)
        return FSMStateName::STAND;
    else if ( user == UserValue::SIT_DOWN)
        return FSMStateName::SIT_DOWN;
    
    return FSMStateName::BALANCE;
}

void State_BalanceTest::calcTau(){
    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _fstate_ctrl->_ioros->_state._imu.getGyroGlobal());

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    // std::cout<<"_forceFeetBody: \n"<<_forceFeetBody<<std::endl;

    _q = vec34ToVec12(_fstate_ctrl->_ioros->getQ());
    _tau = getTau(_q, _forceFeetBody);
    // std::cout<<"_tau: \n"<<_tau<<std::endl;

}