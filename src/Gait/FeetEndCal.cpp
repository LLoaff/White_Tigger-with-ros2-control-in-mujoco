#include "Gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(ControlComponent *ctrlComp)
: _est(ctrlComp->_estimator), _lowState(ctrlComp->_ioros->_state),_robModel(ctrlComp->robotModel){

    _Tstance  = ctrlComp->waveGen->getTstance();
    _Tswing   = ctrlComp->waveGen->getTswing();

    _kx = 0.005;
    _ky = 0.005;
    _kyaw = 0.005;

    Vec34 feetPosBody = _robModel->getFeetPosIdeal();

    for(int i(0); i<4; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2));
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
}

Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase){
    _bodyVelGlobal = _est->getVelocity();
    _bodyWGlobal = _lowState->_imu.getGyroGlobal();
    

    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(_bodyVelGlobal(0) - vxyGoalGlobal(0));
    _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing + _bodyVelGlobal(1)*_Tstance/2 + _ky*(_bodyVelGlobal(1) - vxyGoalGlobal(1));
    _nextStep(2) = 0;
    // double stepX = _bodyVelGlobal(0) *((1.0 - phase) * _Tswing + _Tstance / 2.0)+ _kx *(_bodyVelGlobal(0) - vxyGoalGlobal(0));
    // double stepY =_bodyVelGlobal(1) *((1.0 - phase) * _Tswing + _Tstance / 2.0)+ _ky *(_bodyVelGlobal(1) - vxyGoalGlobal(1));
    // double stepX =vxyGoalGlobal(0) * (1.0 - phase) * _Tswing+ _bodyVelGlobal(0) * _Tstance / 2.0+ _kx * (_bodyVelGlobal(0) - vxyGoalGlobal(0));
    // double stepY =vxyGoalGlobal(1) * (1.0 - phase) * _Tswing+ _bodyVelGlobal(1) * _Tstance / 2.0+ _ky * (_bodyVelGlobal(1) - vxyGoalGlobal(1));

    // constexpr double maxStepX = 0.040;
    // constexpr double maxStepY = 0.025;

    // _nextStep(0) =saturation(stepX,Vec2(-maxStepX, maxStepX));

    // _nextStep(1) =saturation(stepY,Vec2(-maxStepY, maxStepY));
    // _nextStep(2) = 0.0;

    _yaw = _lowState->_imu.getYaw();
    _dYaw = _lowState->_imu.getDYaw();
    _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);

    // _nextYaw =_dYaw *((1.0 - phase) * _Tswing + _Tstance / 2.0)+ _kyaw *(dYawGoal - _dYaw);
    // constexpr double maxYawStep = 0.10;
    // _nextYaw =saturation(static_cast<double>(_nextYaw),Vec2(-maxYawStep, maxYawStep));

    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    _footPos = _est->getPosition() + _nextStep;
    _footPos(2) = 0.0;

    return _footPos;
}

Vec3 FeetEndCal::calFootPos_MIT(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase){

    Vec3 postion = _est->getPosition();
    Vec3 vWorld = _est->getVelocity();
    Vec3 vGoalWorld,vGoalBody;
    vGoalWorld << vxyGoalGlobal(0), vxyGoalGlobal(1), 0.0;
    vGoalBody =  _lowState->_imu.GetRotMat().transpose() * vGoalWorld;
    Vec3 _offset(0,_robModel->getOffset()(legID),0); 

    Vec3 pRobotFrame = _robModel->getHipPos().col(legID) + _offset; // 获取这个腿hip的xyz Body系
    Vec3 pYawCorrected = rotz(-dYawGoal*_Tstance/2) * pRobotFrame;    

    Vec3 Pf = postion + _lowState->_imu.GetRotMat() * (pYawCorrected+vGoalBody*(1-phase)*_Tswing);
    
    float p_rel_max = 0.15; // 最大修正值
    float pfx_rel = vWorld(0)*_Tstance/2 + 0.03* (vWorld(0) - vGoalWorld(0))+(0.5*postion(2)/9.81)*vWorld(1)*dYawGoal;
    float pfy_rel = vWorld(1)*_Tstance/2  + 0.03* (vWorld(1) - vGoalWorld(1))+(0.5*postion(2)/9.81)*-vWorld(0)*dYawGoal;
    
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

    Pf(0) += pfx_rel;
    Pf(1) += pfy_rel;
    Pf(2) = 0.002;
    return Pf;
}
