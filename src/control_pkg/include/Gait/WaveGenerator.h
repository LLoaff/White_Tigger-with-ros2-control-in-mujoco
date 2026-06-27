#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "mathTypes.h"
#include "TimeMaker.h"
#include "FSM/EnumClassList.h"
#include <unistd.h>
// #include "Kenimatics_normal_solution.h"

/*generate linear wave, [0, 1]*/
class WaveGenerator{
public:
    WaveGenerator(double period, double stancePhaseRatio, Vec4 bias);// period:步态周期 stancePhaseRatio：触地系数 bias：偏移时间b与p的比值
    void calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status);//计算相位和接触状态
    float getTstance();// 返回触地时长
    float getTswing(); // 返回腾空时长
    float getT();      // 返回步态周期P
    void reset(double period, double stancePhaseRatio, Vec4 bias);
    Eigen::Matrix<double, 3, 4> _FootPos; // 上一次步态切换的时间点，单位：秒

    const Eigen::Vector4d& getTsw() const;//返回摆动的剩余归一时间
private:
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

    double _period;                 // 周期 P
    double _stRatio;                // 触地系数 r
    Vec4 _bias;                     // 偏移系数

    Vec4 _normalT;                  // [0, 1)  归一化时间 :一个步态周期 0.5 秒，当实际时间过了 0.25 秒时，_normalT = 0.5
    Vec4 _phase, _phasePast;        // _phase：相位 pt= （t-t0）/（t1-t0）支撑 / 摆动相局部相位”（[0,1)，  _phasePast：上一时刻归一化相位
    VecInt4 _contact, _contactPast;
    VecInt4 _switchStatus;          // 1: switching, 0: do not switch 切换标志位（1= 该腿还在切换中，0= 该腿切换完成）
    WaveStatus _statusPast;            

    double _passT;                  // unit: second 从 初始化类开始到调用calcWave经过了的时间
    long long _startT;              // unit: us   记录类初始化的时间

    Eigen::Vector4d    _tsw;                 // 摆动的剩余归一时间
};

#endif
