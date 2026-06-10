#ifndef MPC_H
#define MPC_H

#include "mathtool.h"
#include <vector>
#include <qpOASES.hpp>
#include "MPC/CTP.h"
#include "Estimator.h"
#include "Gait/WaveGenerator.h"
#include <thread>

class mpc
{
public:
    mpc(CTP * ctp,Estimator* est,WaveGenerator* wave,Eigen::Matrix<int,4,1>& contact,bool&_is_wbc_run);
    ~mpc(); 
    void update();
    void reset();

    Eigen::MatrixXd Umpc; // 保留用于日志或调试
    float fri = 0.4f; // 摩擦系数
    float Fmax = 250.0f;

private:
    Eigen::MatrixXd Temp_A,A,Continue_B;
    Eigen::Matrix3d BInertia,PInertia;

    // 后续会在update中resize
    Eigen::MatrixXd Q, R, Aqp, Bqp, D, H, g, lb, ub, lba, uba, Ampc;

    Eigen::VectorXd vec;
    Eigen::MatrixXd temp;
    Eigen::VectorXd Umpc_st;     // 专门用于传递给 WBC 的 3*nst 维向量

    int n_st = 0;                // 当前支撑腿数量

    Eigen::Vector4d MPCsFai;
    
    qpOASES::QProblem *qp_solver = nullptr;
    int last_n_vars = 0;

    Eigen::Matrix<int,4,1>& _contact;
    CTP*                    _ctp;
    Estimator*              _est;
    WaveGenerator*          _wave;

    std::thread               _mpc_thread;
    std::atomic<bool> _running{false}; // 原子退出标志
    void MPC_Loop();
    bool&                    _is_wbc_run;

};

#endif