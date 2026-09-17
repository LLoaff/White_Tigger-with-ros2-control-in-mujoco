#ifndef FREE_STAND_STATE_H
#define FREE_STAND_STATE_H

#include "ControlComponent.h"
#include "FSMState.h"
#include "EnumClassList.h"
#include "math/Kenimatics_normal_solution.h"
#include "math/mathtool.h"
#include "math/Reversal_solution.h"

class Free_Stand_State: public FSMState
{
public:
    Free_Stand_State(ControlComponent * free_stand_ctrl_comp);
    ~Free_Stand_State();
    Eigen::Matrix<double,3,4> Cal_PosinB(double roll,double pitch,double yaw,double height);
    void CalQ(Eigen::Matrix<double,3,4> pos);
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
    Eigen::Matrix<double,3,1>    _pb1_pos_b;   // pb1 在{b}下的坐标
    Eigen::Matrix<double,3,4>    _pos_s;       // pb在{s}下的坐标
    Eigen::Matrix<double,12,1>    _target_speed;  

};


#endif