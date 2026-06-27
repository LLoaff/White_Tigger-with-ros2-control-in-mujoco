#include "Gait/GaitGenerator.h"
GaitGenerator::GaitGenerator(ControlComponent *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->_estimator), 
                _phase(ctrlComp->_phase), _contact(ctrlComp->_contact), 
                 _state(&ctrlComp->_ioros->_state),_robModel(ctrlComp->robotModel),
                 _dt(ctrlComp->dt){
    _feetCal = new FeetEndCal(ctrlComp);
    _firstRun = true;
}

void GaitGenerator::setGait(Vec2 vxyGoalBody, float dYawGoal, float gaitHeight,RotMat G2B){
    _vxyGoal = vxyGoalBody;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart(){
    _firstRun = true;
    _vxyGoal.setZero();
    _phasePast.setZero();
    _contactPast.setOnes();
}

void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel,double period,double stancePhaseRatio,FSMStateName state_name){
    const double Tst = period * stancePhaseRatio;

    if(_firstRun){
        if(state_name == FSMStateName::TROTTING){
            _startP = _robModel->getFeetPosIdeal();

        }
        else if(state_name == FSMStateName::JUMP)
            _startP = _robModel->getFeetPosJump();
        _contactPast = *_contact;
        _firstRun = false;
    }

    for(int i(0); i<4; ++i){

        const Vec3 idealP = (state_name == FSMStateName::JUMP) ? _robModel->getFeetPosJump().col(i)
                                                               : _robModel->getFeetPosIdeal().col(i);
        if((*_contact)(i) == 1){
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
            const double stanceScale = 0.5 - (*_phase)(i);
            feetPos.col(i)(0) = idealP(0) + stanceScale * _vxyGoal(0) * Tst
                                          - stanceScale * _dYawGoal * Tst * idealP(1);
            feetPos.col(i)(1) = idealP(1) + stanceScale * _vxyGoal(1) * Tst
                                          + stanceScale * _dYawGoal * Tst * idealP(0);
            feetPos.col(i)(2) = idealP(2);

            feetVel.col(i)(0) = -_vxyGoal(0) + _dYawGoal * idealP(1);
            feetVel.col(i)(1) = -_vxyGoal(1) - _dYawGoal * idealP(0);
            feetVel.col(i)(2) = 0.0;

            // feetPos.col(i)(0) = _startP.col(i)(0) - _vxyGoal(0) * Tst * (*_phase)(i);
            // feetPos.col(i)(1) = _startP.col(i)(1) - _vxyGoal(1) * Tst * (*_phase)(i);
            // feetVel.col(i)(2) = 0.0f;
        }
        else{
            _startP.col(i)(0) = idealP(0) - 0.5 * _vxyGoal(0) * Tst
                                          + 0.5 * _dYawGoal * Tst * idealP(1);
            _startP.col(i)(1) = idealP(1) - 0.5 * _vxyGoal(1) * Tst
                                          - 0.5 * _dYawGoal * Tst * idealP(0);
            _startP.col(i)(2) = idealP(2);

            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i), period, stancePhaseRatio); // 获得最终x、y的落脚点

            // feetPos.col(i) = getFootPos(i); // 获取下一刻 足端的位置
            // feetVel.col(i) = getFootVel(i); // 获取下一刻 足端的速度

            // _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i),period, stancePhaseRatio);
            
            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
    }
    // std::cout<<"_startP:\n"<< _startP<<std::endl;
    // std::cout<<"_endP:\n"<< _endP<<std::endl;
    // std::cout<<"feetPos:\n"<< feetPos<<std::endl;
    // std::cout<<"feetVel:\n"<< feetVel<<std::endl;
    // usleep(500000);
    _pastP = feetPos;
    _phasePast = *_phase;
    _contactPast = *_contact;
}

Vec3 GaitGenerator::getFootPos(int i){
    // Vec3 footPos;
    // footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    // footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    // footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    // return footPos;
    Vec3 footPos;
    float Tsw = _waveG->getTswing();
    Vec2 vSwing = -_vxyGoal; // 离地与落地的速度
    float aSwing = 0.0f;     // 离地与落地的加速度
    footPos(0) = quinticPolyPosition(_startP.col(i)(0), _endP.col(i)(0), 
                                    vSwing(0), vSwing(0), aSwing, aSwing, (*_phase)(i), Tsw);
    footPos(1) = quinticPolyPosition(_startP.col(i)(1), _endP.col(i)(1), 
                                    vSwing(1), vSwing(1), aSwing, aSwing, (*_phase)(i), Tsw);
    footPos(2) = quinticPolyZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i), Tsw);
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    // Vec3 footVel;
    // footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    // footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    // footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));
    // return footVel;

    Vec3 footVel;
    float Tsw = _waveG->getTswing();
    Vec2 vSwing = -_vxyGoal;
    float aSwing = 0.0f;
    footVel(0) = quinticPolyVelocity(_startP.col(i)(0), _endP.col(i)(0), 
                                    vSwing(0), vSwing(0), aSwing, aSwing, (*_phase)(i), Tsw);
    footVel(1) = quinticPolyVelocity(_startP.col(i)(1), _endP.col(i)(1), 
                                    vSwing(1), vSwing(1), aSwing, aSwing, (*_phase)(i), Tsw);
    footVel(2) = quinticPolyZVelocity(_gaitHeight, (*_phase)(i), Tsw);
    return footVel;
}

float GaitGenerator::cycloidXYPosition(float start, float end, float phase){ // 对应9.11 xy方向位移坐标公式
    float phasePI = 2 * M_PI * phase; // phase == t/T
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){//  对应9.13 xy方向速度计算坐标公式
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

float GaitGenerator::cycloidZPosition(float start, float h, float phase){ // 对应9.11 z方向位移坐标公式
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

float GaitGenerator::cycloidZVelocity(float h, float phase){//  对应9.13 z方向速度计算坐标公式
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}

float GaitGenerator::quinticPolyPosition(float start, float end, float vStart, float vEnd, 
                                        float aStart, float aEnd, float phase,float Tsw) {
    float t = phase * Tsw;
    float T2 = Tsw * Tsw;
    float T3 = T2 * Tsw;
    float T4 = T3 * Tsw;
    float T5 = T4 * Tsw;
    
    float a0 = start;
    float a1 = vStart;
    float a2 = aStart / 2.0f;
    float a3 = (20*(end - start) - (8*vEnd + 12*vStart)*Tsw - (3*aStart - aEnd)*T2) / (2*T3);
    float a4 = (30*(start - end) + (14*vEnd + 16*vStart)*Tsw + (3*aStart - 2*aEnd)*T2) / (2*T4);
    float a5 = (12*(end - start) - (6*vEnd + 6*vStart)*Tsw - (aStart - aEnd)*T2) / (2*T5);
    
    return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
}

float GaitGenerator::quinticPolyVelocity(float start, float end, float vStart, float vEnd, 
                                        float aStart, float aEnd, float phase,float Tsw) {
    float t = phase * Tsw; 
    float T2 = Tsw * Tsw;
    float T3 = T2 * Tsw;
    float T4 = T3 * Tsw;
    float T5 = T4 * Tsw;
    
    float a1 = vStart;
    float a2 = aStart;
    float a3 = 3*(20*(end - start) - (8*vEnd + 12*vStart)*Tsw - (3*aStart - aEnd)*T2) / (2*T3);
    float a4 = 4*(30*(start - end) + (14*vEnd + 16*vStart)*Tsw + (3*aStart - 2*aEnd)*T2) / (2*T4);
    float a5 = 5*(12*(end - start) - (6*vEnd + 6*vStart)*Tsw - (aStart - aEnd)*T2) / (2*T5);
    
    return a1 + a2*t + a3*t*t + a4*t*t*t + a5*t*t*t*t;
}

float GaitGenerator::quinticPolyZPosition(float start, float h, float phase,float Tsw) {
    float t;
    if (phase <= 0.5f) {
        t = phase;
        return start + h * (80.0f*t*t*t - 240.0f*t*t*t*t + 192.0f*t*t*t*t*t);
    } else {
        t = 1.0f - phase;
        return start + h * (80.0f*t*t*t - 240.0f*t*t*t*t + 192.0f*t*t*t*t*t);
    }
}

float GaitGenerator::quinticPolyZVelocity(float h, float phase, float Tsw) {
    float t;
    float sign = 1.0f;
    
    if (phase <= 0.5f) {
        t = phase;
    } else {
        t = 1.0f - phase;
        sign = -1.0f;
    }
    
    float vel = h * (240.0f*t*t - 960.0f*t*t*t + 960.0f*t*t*t*t);
    return vel * sign / Tsw;  // 只在最后除以一次Tsw！
}
