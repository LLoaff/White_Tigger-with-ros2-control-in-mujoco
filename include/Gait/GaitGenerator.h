
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "Gait/WaveGenerator.h"
#include "Gait/FeetEndCal.h"



    /*计算摆线轨迹*/
class GaitGenerator{
public:
    GaitGenerator(ControlComponent *ctrlComp);
    void setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight);
    void run(Vec34 &feetPos, Vec34 &feetVel,double period,double stancePhaseRatio,FSMStateName state_name);
    Vec3 getFootPos(int i);
    Vec3 getFootVel(int i);
    void restart();

    // 五次多项式轨迹：
    float quinticPolyPosition(float start, float end, float vStart, float vEnd, 
                             float aStart, float aEnd, float phase, float Tsw);
    float quinticPolyVelocity(float start, float end, float vStart, float vEnd, 
                             float aStart, float aEnd, float phase, float Tsw);
    float quinticPolyZPosition(float start, float h, float phase, float Tsw);
    float quinticPolyZVelocity(float h, float phase, float Tsw);
private:
    float cycloidXYPosition(float startXY, float endXY, float phase);
    float cycloidXYVelocity(float startXY, float endXY, float phase);
    float cycloidZPosition(float startZ, float height, float phase);
    float cycloidZVelocity(float height, float phase);

    WaveGenerator *_waveG;
    Estimator *_est;
    FeetEndCal *_feetCal;
    LowState *_state;

    float _gaitHeight;
    Vec2 _vxyGoal;
    float _dYawGoal;
    Vec4 *_phase, _phasePast;
    VecInt4 *_contact;
    Vec34 _startP, _endP, _idealP, _pastP;
    QuadrupedRobot *_robModel;

    bool _firstRun;
    float _dt;
    VecInt4 _contactPast; 
};
#endif