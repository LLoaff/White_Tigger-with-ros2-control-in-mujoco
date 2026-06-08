#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include <eigen3/Eigen/Dense>
#include <QuadProg++/QuadProg++.hh>
#include "mathtool.h"

class BalanceCtrl
{
public:
    BalanceCtrl();
    Eigen::Matrix<double, 3, 4> calF(Eigen::Matrix<double, 3, 1> ddPcd, Eigen::Matrix<double, 3, 1> dWbd, 
    Eigen::Matrix<double, 3, 3> rotM, Eigen::Matrix<double, 3, 4> feetPos2B, Eigen::Matrix<int, 4, 1> contact);

private:

    void calMatrixA(Eigen::Matrix<double, 3, 4> feetPos2B, Eigen::Matrix<double, 3, 3> rotM, Eigen::Matrix<int, 4, 1> contact);// 计算动力学方程左侧
    void calVectorBd(Eigen::Matrix<double, 3, 1> ddPcd, Eigen::Matrix<double, 3, 1> dWbd, Eigen::Matrix<double, 3, 3> rotM);//计算动力学方程bd
    void calConstraints(Eigen::Matrix<int, 4, 1> contact);// 化整为求解器格式
    void solveQP();// 调用求解器
    Eigen::Matrix<double, 12, 12> _G, _W, _U;   // _G解算器     _W足端力大小权重    _U足端力改变量权重
    Eigen::Matrix<double, 6, 6> _S;            // 动力学方程权重
    Eigen::Matrix<double, 3, 3> _Ib;           // 惯性张量
    Eigen::Matrix<double, 6, 1> _bd;           // 动力学方程右侧矩阵
    Eigen::Matrix<double, 3, 1> _g;        // 重力加速度
    Eigen::Matrix<double, 3, 1> _pcb;      // 质心在机身坐标系下的位置向量
    Eigen::Matrix<double, 12, 1> _F, _Fprev, _g0T; // _g0T ： _g0的转置
    double _mass, _alpha, _beta, _fricRatio;    // _mass质量   _alpha_W权重系数   _beta _U权重系数 _fricRatio摩擦系数
    Eigen::MatrixXd _CE, _CI;
    Eigen::VectorXd _ce0, _ci0;
    Eigen::Matrix<double, 6 , 12> _A;           // 动力学方程左侧矩阵
    Eigen::Matrix<double, 5 , 3 > _fricMat;     // 摩擦四棱柱约束矩阵

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;
};

#endif
