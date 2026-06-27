#include "Gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(ControlComponent *ctrlComp)
: _est(ctrlComp->_estimator), _lowState(&ctrlComp->_ioros->_state),_robModel(ctrlComp->robotModel){

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

Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyBody, float dYawBody, float phase,double period,double stancePhaseRatio){

    double Kvl = 1.0;
    double Tsw = period * (1-stancePhaseRatio);
    double Tst = period * stancePhaseRatio;

    double _x_line = Kvl*vxyBody(0) * Tsw + 0.5*vxyBody(0)*Tst;
    double _y_line = Kvl*vxyBody(1) * Tsw + 0.5*vxyBody(1)*Tst;

    double _x_yaw = -0.5*dYawBody *Tst*_robModel->getFeetPosIdeal()(1,legID);
    double _y_yaw = 0.5*dYawBody *Tst*_robModel->getFeetPosIdeal()(0,legID);

    _nextStep(0) = _robModel->getFeetPosIdeal()(0,legID)+_x_line+_x_yaw;
    _nextStep(1) = _robModel->getFeetPosIdeal()(1,legID)+_y_line+_y_yaw;
    _nextStep(2) = 0;

    _footPos = _nextStep;
    // double Tst = period * stancePhaseRatio;

    // double _x_line = 0.5*vxyBody(0)*Tst;
    // double _y_line = 0.5*vxyBody(1)*Tst;

    // double _x_yaw = -0.5*dYawBody *Tst*_robModel->getFeetPosIdeal()(1,legID);
    // double _y_yaw = 0.5*dYawBody *Tst*_robModel->getFeetPosIdeal()(0,legID);

    // _nextStep(0) = _robModel->getFeetPosIdeal()(0,legID)+_x_line+_x_yaw;
    // _nextStep(1) = _robModel->getFeetPosIdeal()(1,legID)+_y_line+_y_yaw;
    // _nextStep(2) = 0;

    // _footPos = _nextStep;

    // std::cout<<"_footPos:\n"<< _footPos<<std::endl;


    // _bodyVelGlobal = _est->getVelocity();
    // _bodyWGlobal = _lowState->_imu.getGyroGlobal().cast<double>();
    // _bodyVelGlobal(0) = saturation(_bodyVelGlobal(0), Vec2(vxyGoalGlobal(0) - 0.1, vxyGoalGlobal(0) + 0.1));
    // _bodyVelGlobal(1) = saturation(_bodyVelGlobal(1), Vec2(vxyGoalGlobal(1) - 0.1, vxyGoalGlobal(1) + 0.1));
    // _bodyVelGlobal(2) = 0;

    // _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(_bodyVelGlobal(0) - vxyGoalGlobal(0));
    // _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing + _bodyVelGlobal(1)*_Tstance/2 + _ky*(_bodyVelGlobal(1) - vxyGoalGlobal(1));
    // _nextStep(2) = 0;

    // _yaw = _lowState->_imu.getYaw();
    // _dYaw = _lowState->_imu.getDYaw();
    // _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);

    // _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    // _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    // _footPos = _est->getPosition() + _nextStep;
    // _footPos(2) = 0.0;

    return _footPos;
}
