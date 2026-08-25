#ifndef CONVEXMPC
#define CONVEXMPC

#include "math/mathtool.h"
#include "math/mathTypes.h"
#include "FSM/ControlComponent.h"
#include "Gait/GaitGenerator.h"

class ConvexMPC
{
public:
    ConvexMPC(ControlComponent* comp);
    ~ConvexMPC();

    void MPCrun();
private:
    Mat3 _KpSwing, _KdSwing;
    Mat3 Kp, Kd, Kp_stance, Kd_stance;

    GaitGenerator*  _gait;

    double _dt;                 // 控制器运行周期
    int iterationsBetweenMPC;   // MPC运行周期
    int horizonLength;          // 预测段长度
    ControlComponent* _comp;
    double stand_traj[6];       // 站立姿态
    Vec3 world_position_desired;
    double trajAll[12*36];
    bool firstRun;
};






#endif
