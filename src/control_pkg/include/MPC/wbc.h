#ifndef WBC_H
#define WBC_H
#include <iostream>
#include <vector>
#include "mathtool.h"
#include <memory>
#include "Gait/FeetEndCal.h"
#include "MPC/CTP.h"
#include "MPC/mpc.h"
#include "Estimator.h"
#include "Gait/WaveGenerator.h"
#include "low/LowState.h"
#include "IOPort/IORos.h"
#include <thread>
#include <QuadProg++/QuadProg++.hh>
#ifdef inverse
#undef inverse
#endif
struct node
{
    std::weak_ptr<node> parent;
    std::vector<std::shared_ptr<node>> child;
    int const num;
    node(int const n) : num(n) {}
    void add_child(const std::shared_ptr<node> &c)
    {
    child.push_back(c);
    }
};

class wbc
{
public:
    wbc(Estimator* est,LowState* lowstate,FeetEndCal* feetcal,CTP* ctp,mpc* mpc,Eigen::Matrix<int,4,1>& contact,const std::shared_ptr<IORos>& ioros,bool& _is_wbc_run);
    void bind(std::shared_ptr<node> &parent, std::shared_ptr<node> &child);
    void CreatTree();
    Eigen::MatrixXd Transform_P2C(std::shared_ptr<node> node);
    Eigen::MatrixXd Transform_C2P(std::shared_ptr<node> node);
    Eigen::MatrixXd Transform_C2PF(std::shared_ptr<node> node);
    // 阻尼最小二乘法
    Eigen::MatrixXd WideInverse(const Eigen::MatrixXd &mat);
    Eigen::MatrixXd Vcross(const Eigen::MatrixXd &V);
    Eigen::MatrixXd Fcross(const Eigen::MatrixXd &V);

    void WBC_Reset();
    void Dynamcis_Update();
    void WBC_Update();

private:
    Estimator*  _est;
    Imu*        _imu;
    LowState*   _lowstate;
    CTP*        _ctp;
    mpc*        _mpc;
    std::shared_ptr<IORos> _ioros;
    
    Eigen::Matrix<double, 3, 4>&     _FootdesirePos,_FootdesireVelocity;
    Eigen::Matrix<double,3,4>&   iPb;
    Eigen::Matrix<int, 4, 1>&     _contact;
    Eigen::Matrix<int,4,1> con; 
    bool is_tree_created = false;// 安全锁

    //  广义qdot qddot
    Eigen::MatrixXd qdot,qddot, qcmd, qdotcmd,
        qddotcmd, detqcmd, q, qdotcmde, qddotcmde;

    // 4个任务雅可比矩阵
    Eigen::MatrixXd J1, J2, J3, J4, J1q, J2q
        ,J3q, J4q, e, x, JA, NA;
    Eigen::Matrix3d tran2; // 任务2 用的
    Eigen::MatrixXd Q1, Q2, G, CE, Ce, CI
        ,Ci, Mf, Jcf, Cf, CA, _CA, CA_;

    float kp = 150.f;
    float kd = 100.f;

    // 空间速度 空间加速度 {0} 到任意坐标系的变换矩阵   Si从 i=1 开始
    std::vector<Eigen::MatrixXd> Vspace, Aspace, Aspaced, X02I
        ,X02If, Si, fi, Vji, qidot, Jb, XQi, Jbi, AspaceQ, VspaceQ, I, Ic;
    Eigen::MatrixXd XCi, C, M;
    std::vector<int> pi;

    std::shared_ptr<node> node0 = std::make_shared<node>(0);
    std::shared_ptr<node> node1 = std::make_shared<node>(1);
    std::shared_ptr<node> node2 = std::make_shared<node>(2);
    std::shared_ptr<node> node3 = std::make_shared<node>(3);
    std::shared_ptr<node> node4 = std::make_shared<node>(4);
    std::shared_ptr<node> node5 = std::make_shared<node>(5);
    std::shared_ptr<node> node6 = std::make_shared<node>(6);
    std::shared_ptr<node> node7 = std::make_shared<node>(7);
    std::shared_ptr<node> node8 = std::make_shared<node>(8);
    std::shared_ptr<node> node9 = std::make_shared<node>(9);
    std::shared_ptr<node> node10 = std::make_shared<node>(10);
    std::shared_ptr<node> node11 = std::make_shared<node>(11);
    std::shared_ptr<node> node12 = std::make_shared<node>(12);
    std::shared_ptr<node> node13 = std::make_shared<node>(13);
    std::vector<std::shared_ptr<node>> Vnode;

    std::thread               _wbc_thread;
    std::atomic<bool> _running{false}; // 原子退出标志
    void WBC_Loop();
    bool&                    _is_wbc_run;

};




#endif