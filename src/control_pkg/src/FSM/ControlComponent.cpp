#include "FSM/ControlComponent.h"

ControlComponent::ControlComponent(){
    _contact = new Eigen::Matrix<int,4,1>();
    _phase = new Eigen::Matrix<double,4,1>();
    *_contact = Eigen::Matrix<int,4,1>(0,0,0,0);
    *_phase = Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.5);
    robotModel = new QuadrupedRobot();     

    // _servo = new Servo(18);
    // _ioros = new LowCmd();
    dt = 0.003;    
    _period = 0.5;
    _stancePhaseRatio = 0.5;

    waveGen = new WaveGenerator(_period, _stancePhaseRatio, Vec4(0, 0.5, 0.5, 0)); // Trot

    _ioros = std::make_shared<IORos>();
    _estimator = new Estimator(&_ioros->_state, _contact,_phase,dt);

    user_cmd = new UserCmd();
    _ctp = new CTP(_estimator,waveGen->_FootPos, dt, user_cmd->_vx, user_cmd->_vy, user_cmd->_wz);

    executor.add_node(_ioros);
    spin_thread = std::thread([this]() {
        executor.spin(); // 子线程在这里阻塞
    });
    std::cout << "IOROS初始化 OK" << std::endl;
}

void ControlComponent::runWaveGen(){
    // Eigen::Matrix<double,3,4> footPos = GetFeetPos2BODY(this->_ioros->_state,FrameType::BODY);
    Eigen::Matrix<double,3,4> footPos = this->_estimator->getFeetPos(); // 获取足端位置 global

    waveGen->calcContactPhase(*_phase, *_contact, _waveStatus, footPos);
    _ctp->update();
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
}

ControlComponent::~ControlComponent(){
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    if (spin_thread.joinable()){
        spin_thread.join();
    }
    delete _estimator;
    delete _contact;
    delete _phase;
    delete robotModel;
    // delete _servo;
    // delete _ioros;
    delete user_cmd;
}
