#ifndef CONTROLCOMPONENT_H
#define CONTROLCOMPONENT_H

#include "UserCmd.h"
#include "Estimator.h"
#include "Gait/WaveGenerator.h"
#include "Robot.h"
#include <thread>
#include "low/LowCmd.h"
#include "IOPort/IORos.h"
#include "CTP.h"

// #include "low/servo.h"
class ControlComponent
{
public:
    ControlComponent();
    ~ControlComponent();

    void runWaveGen();
    void setAllStance();
    void setAllSwing();
    void setStartWave();
    void Estimator_Init();
    
    UserCmd  *  user_cmd; // 获取单一实例
    Estimator * _estimator;
    QuadrupedRobot *robotModel;
    CTP*            _ctp;       
    double dt;
    double _period;
    double _stancePhaseRatio;
    Eigen::Matrix<int,4,1> * _contact;
    Eigen::Matrix<double, 4, 1>* _phase;
    WaveGenerator *waveGen;

    // LowCmd * _ioros;
    std::shared_ptr<IORos>  _ioros;

    // Servo * _servo;
private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;
    std::thread             spin_thread;
    rclcpp::executors::MultiThreadedExecutor executor;

};

#endif