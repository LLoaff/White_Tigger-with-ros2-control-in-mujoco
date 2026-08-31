#include "FSM/ControlComponent.h"

ControlComponent::ControlComponent(mjModel *model, mjData *data):_mjmodel(model)
,_mjdata(data){
    root_body_id = mj_name2id(_mjmodel, mjOBJ_BODY, "root");
    if (root_body_id == -1) {
        std::cerr << "没有找到 root body" << std::endl;
        return;
    }
    _contact = new Eigen::Matrix<int,4,1>();
    _phase = new Eigen::Matrix<double,4,1>();
    *_contact = Eigen::Matrix<int,4,1>(0,0,0,0);
    *_phase = Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.5);
    robotModel = new QuadrupedRobot();     

    _ioros = new LowCmd(model,data);
    user_cmd = new UserCmd();
    _ioros->_state->_imu.Imu_Initial();


}

void ControlComponent::runWaveGen(){
    waveGen->calcContactPhase(*_phase, *_contact, _waveStatus,_mjdata->time );
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
    _estimator = new Estimator(_ioros->_state, _contact,_phase,dt);
    _balCtrl = new BalanceCtrl();
}
WaveStatus ControlComponent::getWaveStatus(){
    return _waveStatus;
}   

ControlComponent::~ControlComponent(){
    delete _estimator;
    delete _contact;
    delete _phase;
    delete _balCtrl;
    delete robotModel;
    delete _ioros;
    delete user_cmd;
}
