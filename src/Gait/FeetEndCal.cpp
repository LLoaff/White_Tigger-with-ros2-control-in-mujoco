#include "Gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(ControlComponent *ctrlComp)
: _est(ctrlComp->_estimator), _lowState(&ctrlComp->_ioros->_state),_robModel(ctrlComp->robotModel){

    _Tstance  = ctrlComp->waveGen->getTstance();
    _Tswing   = ctrlComp->waveGen->getTswing();

    _kx = 0.1;
    _ky = 0.1;
    _kyaw = 0.1;

    Vec34 feetPosBody = _robModel->getFeetPosIdeal();

    for(int i(0); i<4; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2));
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
}

Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase,double period,double stancePhaseRatio){
    
    double Kvl = 0.7;
    double Tsw = period * (1-stancePhaseRatio);
    double Tst = period * stancePhaseRatio;

    double _x_line = Kvl*vxyGoalGlobal(0) * Tsw + 0.5*vxyGoalGlobal(0)*Tst;
    double _y_line = Kvl*vxyGoalGlobal(1) * Tsw + 0.5*vxyGoalGlobal(1)*Tst;

    double _x_yaw = -0.5*dYawGoal *Tst*_robModel->getFeetPosIdeal()(1,legID);
    double _y_yaw = 0.5*dYawGoal *Tst*_robModel->getFeetPosIdeal()(0,legID);

    _nextStep(0) = _robModel->getFeetPosIdeal()(0,legID)+_x_line+_x_yaw;
    _nextStep(1) = _robModel->getFeetPosIdeal()(1,legID)+_y_line+_y_yaw;
    _nextStep(2) = 0;

    _footPos = _nextStep;
    // std::cout<<"_footPos:\n"<< _footPos<<std::endl;

    return _footPos;
}
