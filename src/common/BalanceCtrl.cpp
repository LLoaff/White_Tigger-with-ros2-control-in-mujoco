#include "BalanceCtrl.h"

BalanceCtrl::BalanceCtrl(){
    Eigen::Matrix<double, 6 , 1> s;
    Eigen::Matrix<double, 12 , 1> w,u;
    // _mass = 12.7;
    _mass = 19.7;

    // _pcb<< 0.028,0.002,0;
    _pcb<< 0,0,0;

    _Ib<< 0.542828,0.141737,0.170247,
          0.141737,0.496037,0.228799,
          0.170247,0.228799,0.498639;
    
    // _Ib<<0.1051041666666,0,        0,
    //      0,              0.3271875,0,
    //      0,              0,        0.3854166666666;
    _g << 0, 0, -9.81;

    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;

    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    s << 20, 20, 50, 550, 550, 550;

    _alpha = 0.1;
    _beta  = 0.5;
    _fricRatio = 0.7;
    _S = s.asDiagonal();
    _W = w.asDiagonal();
    _U = u.asDiagonal();
    
    _Fprev.setZero();
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
}

Eigen::Matrix<double, 3, 4> BalanceCtrl:: calF(Eigen::Matrix<double, 3, 1> ddPcd, Eigen::Matrix<double, 3, 1> dWbd, 
Eigen::Matrix<double, 3, 3> rotM, Eigen::Matrix<double, 3, 4> feetPos2B, Eigen::Matrix<int, 4, 1> contact){ 
    calMatrixA(feetPos2B, rotM, contact);

    calVectorBd(ddPcd, dWbd, rotM);

    calConstraints(contact);

    _G = _A.transpose()*_S*_A + _alpha*_W + _beta*_U;
    // Eigen::Matrix<double, 1, 12> _g0;
    // _g0 = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;
    // _g0T = _g0.transpose();
    _g0T = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;
    solveQP();
    _Fprev = _F;
    // std::cout<<"F:\n"<<_Fprev<<" --- \n"<<std::endl;
    return vec12ToVec34(_F);
}

void BalanceCtrl::calMatrixA(Eigen::Matrix<double, 3, 4> feetPos2B, Eigen::Matrix<double, 3, 3> rotM, Eigen::Matrix<int, 4, 1> contact){
    for(int i(0); i < 4; ++i){

        _A.block(0, 3*i, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

        _A.block(3, 3*i, 3, 3) = skew(feetPos2B.col(i) - rotM*_pcb);    // skew 生成反对称矩阵，用于叉乘
    }
}   

void BalanceCtrl::calVectorBd(Eigen::Matrix<double, 3, 1> ddPcd, Eigen::Matrix<double, 3, 1> dWbd, Eigen::Matrix<double, 3, 3> rotM){
    _bd.head(3) = _mass * (ddPcd - _g);
    _bd.tail(3) = (rotM * _Ib * rotM.transpose()) * dWbd;
}

void BalanceCtrl::calConstraints(Eigen::Matrix<int, 4, 1> contact){
    int contactLegNum = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            contactLegNum += 1;
        }
    }

    _CI.resize(5*contactLegNum, 12);
    _ci0.resize(5*contactLegNum);
    _CE.resize(3*(4 - contactLegNum), 12);
    _ce0.resize(3*(4 - contactLegNum));

    _CI.setZero();
    _ci0.setZero();
    _CE.setZero();
    _ce0.setZero();

    int ceID = 0;
    int ciID = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            _CI.block(5*ciID, 3*i, 5, 3) = _fricMat;
            ++ciID;
        }else{
            _CE.block(3*ceID, 3*i, 3, 3) =Eigen::MatrixXd::Identity(3, 3);
            ++ceID;
        }
    }
}

void BalanceCtrl::solveQP(){
    int n = _F.size();
    int m = _ce0.size();
    int p = _ci0.size();

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = _G(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = (_CE.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = _g0T[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = _ce0[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = _ci0[i];
    }

    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    for (int i = 0; i < n; ++i) {
        _F[i] = x[i];
    }
}