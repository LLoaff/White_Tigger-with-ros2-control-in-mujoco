#include "FSM/ControlComponent.h"

ControlComponent::ControlComponent(){
    _contact = new Eigen::Matrix<int,4,1>();
    _phase = new Eigen::Matrix<double,4,1>();
    *_contact = Eigen::Matrix<int,4,1>(0,0,0,0);
    *_phase = Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.5);
    robotModel = new QuadrupedRobot();     

    // _servo = new Servo(18);
    _ioros = new LowCmd();
    user_cmd = new UserCmd();
    _ioros->_state._imu.Imu_Initial();
}

void ControlComponent::runWaveGen(){
    waveGen->calcContactPhase(*_phase, *_contact, _waveStatus);
}

void ControlComponent::setAllStance(){
    _waveStatus = WaveStatus::STANCE_ALL;
}

void ControlComponent::setAllSwing(){
    _waveStatus = WaveStatus::SWING_ALL;
}

void ControlComponent::setStartWave(){
    _waveStatus = WaveStatus::WAVE_ALL;
}

void ControlComponent::Estimator_Init(){
    _estimator = new Estimator(&_ioros->_state, _contact,_phase,dt);
    balCtrl = new BalanceCtrl();
}

ControlComponent::~ControlComponent(){
    delete _estimator;
    delete _contact;
    delete _phase;
    delete balCtrl;
    delete robotModel;
    // delete _servo;
    delete _ioros;
    delete user_cmd;
}
