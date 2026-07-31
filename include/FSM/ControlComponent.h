#ifndef CONTROLCOMPONENT_H
#define CONTROLCOMPONENT_H

#include "FSM/UserCmd.h"

#include "Gait/WaveGenerator.h"
#include "math/Robot.h"
#include <thread>
#include "low/LowCmd.h"
#include "WBC/BalanceCtrl.h"
#include "WBC/Estimator.h"

class ControlComponent{
public:
    ControlComponent(mjModel *model, mjData *data);
    ~ControlComponent();

    void runWaveGen();
    void setAllStance();
    void setAllSwing();
    void setStartWave();
    void Estimator_Init();
    
    UserCmd  *  user_cmd; // 获取单一实例
    Estimator * _estimator;
    BalanceCtrl* _balCtrl;
    QuadrupedRobot *robotModel;

    double dt;
    double _period;
    double _stancePhaseRatio;
    Eigen::Matrix<int,4,1> * _contact;
    Eigen::Matrix<double, 4, 1>* _phase;
    WaveGenerator *waveGen;

    LowCmd * _ioros;
private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;
};

#endif