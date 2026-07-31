#include "FSM/ControlComponent.h"

ControlComponent::ControlComponent(){
    _contact = new Eigen::Matrix<int,4,1>();
    _phase = new Eigen::Matrix<double,4,1>();
    *_contact = Eigen::Matrix<int,4,1>(0,0,0,0);
    *_phase = Eigen::Matrix<double,4,1>(0.5,0.5,0.5,0.5);
    robotModel = new QuadrupedRobot();     
    _balCtrl = new BalanceCtrl();

    dt = 0.0025;    
    _period = 0.5;
    _stancePhaseRatio = 0.5;

    waveGen = new WaveGenerator(_period, _stancePhaseRatio, Vec4(0, 0.5, 0.5, 0)); // Trot

    _ioros = std::make_shared<IORos>();

    user_cmd = new UserCmd();

    executor.add_node(_ioros);
    spin_thread = std::thread([this](){
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(0, &cpuset);
        CPU_SET(1, &cpuset);
        CPU_SET(2, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
        executor.spin(); // 子线程在这里阻塞
    });
    std::cout << "IOROS初始化 OK" << std::endl;
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
    delete user_cmd;
    delete _balCtrl;
}
