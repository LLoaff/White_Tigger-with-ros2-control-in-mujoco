#include "WBC/mujocoEstimator.h"

M_Estimator::M_Estimator(LowState* lowstate,mjModel *model,mjData* mjdata,double* boxpos)
:_mjmodel(model),_mjdata(mjdata),_lowstate(lowstate),_box_pos(boxpos){
    X.setZero();
    _root_id = mj_name2id(_mjmodel,mjOBJ_BODY,"root");
    if(_root_id == -1){
        std::cout<<"没有 root"<<std::endl;
    }

}

void M_Estimator::init(){

}

void M_Estimator::run(){
    X(0) = _mjdata->xpos[_root_id*3 + 0];
    X(1) = _mjdata->xpos[_root_id*3 + 1];
    X(2) = _mjdata->xpos[_root_id*3 + 2];   
}

M_Estimator::~M_Estimator(){

}

Eigen::Matrix<double, 3, 1> M_Estimator::getPosition(){
    return X.block(0, 0, 3, 1);

}

Eigen::Matrix<double, 3, 1> M_Estimator::getVelocity(){
    return X.block(3, 0, 3, 1);

}
Eigen::Matrix<double, 3, 1>  M_Estimator::getFootPos(int i){
    return  getPosition() + _lowstate->_imu.GetRotMat().cast<double>() * GetFeetPos2BODY(*_lowstate, i, FrameType::BODY);
}
Eigen::Matrix<double, 3, 4> M_Estimator::getFeetPos(){
    Eigen::Matrix<double, 3, 4> feetPos;
    for(int i(0); i < 4; ++i){
        feetPos.col(i) = getFootPos(i);
    }
    return feetPos;
}

Eigen::Matrix<double, 3, 4> M_Estimator::getFeetVel(){
    Eigen::Matrix<double, 3, 4> feetVel = GetFeetSpeed2BODY(*_lowstate, FrameType::GLOBAL);
    for(int i(0); i < 4; ++i){
        feetVel.col(i) += getVelocity();
    }
    return feetVel;
}

Eigen::Matrix<double, 3, 4> M_Estimator::getPosFeet2BGlobal(){
    return Eigen::Matrix<double, 3, 4>::Zero();
}

void M_Estimator::updateBoxPos(){
    *(_box_pos+0) = X(0);
    *(_box_pos+1) = X(1);
    *(_box_pos+2) = X(2);
}
