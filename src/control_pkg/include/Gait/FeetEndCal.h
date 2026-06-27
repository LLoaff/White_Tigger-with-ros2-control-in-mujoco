#ifndef FEETENDCAL_H
#define FEETENDCAL_H

#include "FSM/ControlComponent.h"
#include "low/LowState.h"
#include "mathTypes.h"
#include "Robot.h"

/*计算落脚点*/
class FeetEndCal{
public:
    FeetEndCal(ControlComponent * ctrlComp);
    Vec3 calFootPos(int legID, Vec2 vxyBody, float dYawBody, float phase,double period,double stancePhaseRatio);
private:
    LowState *_lowState;
    Estimator *_est;

    Vec3 _nextStep, _footPos;
    Vec3 _bodyVelGlobal;        // linear velocity
    Vec3 _bodyAccGlobal;        // linear accelerator
    Vec3 _bodyWGlobal;          // angular velocity

    QuadrupedRobot *_robModel;
    Vec4 _feetRadius, _feetInitAngle; // _feetRadius:以机身原点画圆，该腿所在的半径  _feetInitAngle：进入该模式后 该腿距离头部的角度
    float _yaw, _dYaw, _nextYaw;

    float _Tstance, _Tswing;
    float _kx, _ky, _kyaw;
};
#endif  // FEETENDCAL_H