#include "Gait/WaveGenerator.h"

                                  // 0.4              0.5                 0 ,0.5 ,0.5,0 
WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias)
: _period(period), _stRatio(stancePhaseRatio), _bias(bias){
    if ((_stRatio >= 1) || (_stRatio <= 0))
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0))
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    _startT = getSystemTime();
    _contactPast.setZero();
    _phasePast << 0.5, 0.5, 0.5, 0.5;
    _statusPast = WaveStatus::SWING_ALL;
}

void WaveGenerator::reset(double period, double stancePhaseRatio, Vec4 bias) {
    if ((stancePhaseRatio >= 1) || (stancePhaseRatio <= 0)) {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        return; 
    }

    for (int i(0); i < bias.rows(); ++i) {
        if ((bias(i) > 1) || (bias(i) < 0)) {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            return;
        }
    }

    _period = period;
    _stRatio = stancePhaseRatio;
    _bias = bias;

    _startT = getSystemTime();
    _contactPast.setZero();
    _phasePast << 0.5, 0.5, 0.5, 0.5;
    _statusPast = WaveStatus::SWING_ALL;
    _switchStatus.setZero();
}
// 步态切换控制器 
void WaveGenerator::calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status){
    calcWave(_phase, _contact, status);
    if (status != _statusPast)
    {
        if (_switchStatus.sum() == 0)
        {
            _switchStatus.setOnes();
        }
        calcWave(_phasePast, _contactPast, _statusPast);

        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL))
        {
            _contactPast.setOnes();
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL))
        {
            _contactPast.setZero();
        }
    }

    if (_switchStatus.sum() != 0){
        for (int i(0); i < 4; ++i)
        {
            if (_contact(i) == _contactPast(i))
            {
                _switchStatus(i) = 0;//  如果“目标接触”和“上一接触”一样：这条腿不用动，切换完成
            }
            else
            {       
                _contact(i) = _contactPast(i);  // 如果“目标接触”和“上一接触”不一样：强制保持上一状态，不跳变
                _phase(i) = _phasePast(i);
            }
        }
        if (_switchStatus.sum() == 0)
        {
            _statusPast = status;
        }
    }

    phaseResult = _phase;
    contactResult = _contact;
}
// 返回触地时长
float WaveGenerator::getTstance(){
    return _period * _stRatio;
}
// 返回腾空时长
float WaveGenerator::getTswing(){
    return _period * (1 - _stRatio);
}
// 返回步态周期P
float WaveGenerator::getT(){
    return _period;
}

void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status){
    if (status == WaveStatus::WAVE_ALL){
        _passT = (double)(getSystemTime() - _startT) * 1e-6; 
        for (int i(0); i < 4; ++i){
                                // fmod:求余函数fmod(x,y): x/y的余数  _period * _bias(i): 偏移时间  
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period;
            if (_normalT(i) < _stRatio){                    // 时间轴（_normalT）:  0.0  0.2  0.4  0.6  0.8  1.0
                contact(i) = 1;                             //                      |----|----|----|----|----|
                phase(i) = _normalT(i) / _stRatio;          // RF(0) 状态:          [  支撑相     ][  摆动相  ]
            }                                               // LF(1) 状态:          [  摆动相  ][  支撑相     ]
            else{                                           // (相位差0.5)
                contact(i) = 0;
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);   // 支撑相区间：_normalT ∈ [0, _stRatio)
            }                                                           // 摆动相区间：_normalT ∈ [_stRatio, 1)
        }                                                               // phase(i) = _normalT(i) / _stRatio; ： phase = 0：支撑相开始（刚落地）
    }                                                                   // phase = 1：支撑相结束（准备抬腿）
    else if (status == WaveStatus::SWING_ALL){
        contact.setZero();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
    else if (status == WaveStatus::STANCE_ALL){
        contact.setOnes();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
}