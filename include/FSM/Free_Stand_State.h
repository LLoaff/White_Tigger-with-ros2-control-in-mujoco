#ifndef FREE_STAND_STATE_H
#define FREE_STAND_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "Kenimatics_normal_solution.h"
#include "mathtool.h"
#include "Reversal_solution.h"

class Free_Stand_State: public FSMState
{
public:
    Free_Stand_State(ControlComponent * free_stand_ctrl_comp);
    ~Free_Stand_State();
    Eigen::Matrix<float,3,4> Cal_PosinB(float roll,float pitch,float yaw,float height);
    void CalQ(Eigen::Matrix<float,3,4> pos);
    void enter();
    void run();
    void exit();
    FSMStateName CheckChange();
private:
    float        _rowMax;
    float        _rowMin;
    float        _pitchMax;
    float        _pitchMin;
    float        _yawMax;
    float        _yawMin;
    float        _heightMax;
    float        _heightMin;
    Eigen::Matrix<float,3,1>    _pb1_pos_b;   // pb1 在{b}下的坐标
    Eigen::Matrix<float,3,4>    _pos_s;       // pb在{s}下的坐标
    Eigen::Matrix<float,12,1>    _target_speed;  

};


#endif