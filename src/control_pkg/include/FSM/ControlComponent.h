#ifndef CONTROLCOMPONENT_H
#define CONTROLCOMPONENT_H

#include "UserCmd.h"
#include "Estimator.h"
#include "Gait/WaveGenerator.h"
#include "Robot.h"
#include <thread>
#include "low/LowCmd.h"
#include "IOPort/IORos.h"
#include "BalanceCtrl.h"

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

    // FeetEndCal *    _feetendcal;
    BalanceCtrl *_balCtrl;

    double dt;
    double _period;
    double _stancePhaseRatio;
    Eigen::Matrix<int,4,1> * _contact;
    Eigen::Matrix<double, 4, 1>* _phase;
    WaveGenerator *waveGen;

    // LowCmd * _ioros;
    std::shared_ptr<IORos>  _ioros;
    
    bool                    _is_wbc_run=false;

    // Servo * _servo;
private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;
    std::thread             spin_thread;
    pthread_t               _wbc_thread;

    rclcpp::executors::MultiThreadedExecutor executor;

};

#endif