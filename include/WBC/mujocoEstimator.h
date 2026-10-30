#ifndef MUJOCOESTIMATOR_H
#define MUJOCOESTIMATOR_H

#include "low/LowState.h"
#include "eigen3/Eigen/Dense"
#include "math/mathtool.h"
#include "math/mathTypes.h"
// #include "FSM/EnumClassList.h"
#include "math/Kenimatics_normal_solution.h"
#include "mujoco/mujoco.h"

class M_Estimator{
public:
    M_Estimator(LowState* lowstate,mjModel *model,mjData* mjdata,float* boxpos);
    ~M_Estimator();
    void init();
    void run();
    Eigen::Matrix<float, 3, 1> getPosition();
    Eigen::Matrix<float, 3, 1> getVelocity();
    Eigen::Matrix<float, 3, 1> getFootPos(int i);
    Eigen::Matrix<float, 3, 4> getFeetPos();
    Eigen::Matrix<float, 3, 4> getFeetVel();
    Eigen::Matrix<float, 3, 4> getPosFeet2BGlobal();

    void updateBoxPos();
private:
    int _root_id = -1;
    mjModel*  _mjmodel;
    mjData* _mjdata;
    LowState* _lowstate;
    float* _box_pos;
    Eigen::Matrix<int, 4, 1>* _contatc;
    
    Eigen::Matrix<float,18,1> X;


};

#endif