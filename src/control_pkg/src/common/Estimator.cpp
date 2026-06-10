#include"Estimator.h"

Estimator::Estimator(LowState* lowstate,Eigen::Matrix<int, 4, 1>* contact,Eigen::Matrix<double, 4, 1> * phase,double dt):
_lowstate(lowstate),_contact(contact),_phase(phase),_dt(dt){
    init();
}

void Estimator::init(){
    // _g << 0,0,-9.81;

    
    Q = Eigen::MatrixXd::Identity(18, 18);
    iden3 = Eigen::Matrix3d::Identity();
    _3x3  = Eigen::Matrix3d::Zero();
    iden12 = Eigen::MatrixXd::Identity(12, 12);
    iden18 = Eigen::MatrixXd::Identity(18, 18);
    K = Eigen::MatrixXd(18, 28);
    Z = Eigen::MatrixXd::Zero(28, 1);

    _3x12.setZero();
    _12x3.setZero();
    Q.setZero();
    R.setZero();
    
    A << iden3, _dt * iden3, _3x12,
          _3x3, iden3, _3x12,
          _12x3, _12x3, iden12;
    B << _3x3, _dt * iden3, _12x3;
    U << 0, 0, -9.81;

    // _X= Eigen::MatrixXd(18, 1);
    _X.setZero();
    // X= Eigen::MatrixXd(18, 1);
    X.setZero();

    // _P.setIdentity();           // 对角线设为1 ，非对角线设为0
    // _P = _largeVariance * _P;   // 开机时 使其极度信任测量值

    // H = Eigen::MatrixXd(28, 18);
    H << 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
          1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0,
          1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    X << 0,0,0.07912,  0,0,0,  0.21,-0.109,0,  0.21,0.109,0,  -0.155,-0.109,0, -0.155,0.109,0;

    P.setZero();
    P.block(0, 0, 3, 3) = 0.01f * Eigen::Matrix3d::Identity();
    P.block(3, 3, 3, 3) = 0.1f * Eigen::Matrix3d::Identity();
    P.block(6, 6, 12, 12) = 0.01f * Eigen::MatrixXd::Identity(12, 12);

    iden18 = Eigen::MatrixXd::Identity(18, 18);

    Onemat << 1;
    Q.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);



    // _RCheck  = new AvgCov(28,  " R");
    // _uCheck  = new AvgCov(3,   " u");
}

void Estimator::run(){
    _feetPosBodyKine = GetFeetPos2BODY(*_lowstate,FrameType::BODY);
    _feetVelGlobalKine = GetFeetSpeed2BODY(*_lowstate,FrameType::GLOBAL);
    U<<0,0,-9.81;
    // std::cout<<"U: \n"<<U<<std::endl;

    // _Q = _QInit;    // 必须赋值，后面根据步态进行调整
    // _R = _RInit;
    
    for(int i(0);i<4;++i){        
        if((*_contact)(i) == 0){
            Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = iden3;
            R.block(0 + 3 * i, 0 + 3 * i, 3, 3) = iden3 * 20000;
            R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = iden3 * 20000;
            R.block(24 + i, 24 + i, 1, 1) = Onemat * 20000;
        }   
        else{                                                               //_phase：4 只脚的步态相位  
            Q.block(6+3*i, 6+3*i, 3, 3) = iden3;
            R.block(0 + 3 * i, 0 + 3 * i, 3, 3) = iden3;
            R.block(12+3*i, 12+3*i, 3, 3) = iden3;
            R.block(24+i, 24+i, 1, 1) = Onemat;
        }
        _feetPos2Body.segment(3*i,3) = _feetPosBodyKine.col(i);
        _feetVel2Body.segment(3*i,3) = _feetVelGlobalKine.col(i);
    }
    Fripb = _feetPos2Body.segment(0,3);
    Flipb = _feetPos2Body.segment(3,3);
    Rripb = _feetPos2Body.segment(6,3);
    Rlipb = _feetPos2Body.segment(9,3); 
    FriPbv = _feetVel2Body.segment(0,3);
    FliPbv = _feetVel2Body.segment(3,3);
    RriPbv = _feetVel2Body.segment(6,3);
    RliPbv = _feetVel2Body.segment(9,3);
    iPb.col(0) = Fripb;
    iPb.col(1) = Flipb;
    iPb.col(2) = Rripb;
    iPb.col(3) = Rlipb;
    _rotMatB2G = _lowstate->_imu.GetRotMat().cast<double>();
    Z << -_rotMatB2G* Fripb, - _rotMatB2G*Flipb, - _rotMatB2G*Rripb, - _rotMatB2G*Rlipb,
        -_rotMatB2G * FriPbv, -_rotMatB2G * FliPbv, -_rotMatB2G * RriPbv, -_rotMatB2G * RliPbv,
        0, 0, 0, 0;
   
    _X = A * X + B * U;
    _P = A * P * (A.transpose()) + Q;

    Eigen::MatrixXd S = H * _P * H.transpose() + R;
    Eigen::MatrixXd S_inv = S.ldlt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

    K = _P * H.transpose() * S_inv;
    X = _X + K * (Z - H * _X);
    P = (iden18 - K * H) * _P;

    P = 0.5f * (P + P.transpose()); // 强制对称化保护！
    // std::cout<<"p_com: \n"<<X.block(0, 0, 3, 1)<<"\n v_com: \n"<<X.block(3, 0, 3, 1)<<std::endl;
    // std::cout<<"p_com: \n"<<X.block(0, 0, 3, 1)<<std::endl;
    // std::cout<<"p1: \n"<<X.block(6, 0, 3, 1)<<std::endl;


    _pcom = X.block(0, 0, 3, 1);
    _vcom = X.block(3, 0, 3, 1);
}

Eigen::Matrix<double, 3, 1>  Estimator::getPosition(){
    return X.block(0, 0, 3, 1);
}
Eigen::Matrix<double, 3, 1>  Estimator::getVelocity(){
    // Eigen::Matrix<double, 3, 1> v;
    // v<< 0.07,0,0;
    return X.block(3, 0, 3, 1);
    // return v;
}
Eigen::Matrix<double, 3, 1>  Estimator::getFootPos(int i){
    return getPosition() + _lowstate->_imu.GetRotMat().cast<double>() * GetFeetPos2BODY(*_lowstate, i, FrameType::BODY);
}
Eigen::Matrix<double, 3, 4> Estimator::getFeetPos(){
    Eigen::Matrix<double, 3, 4> feetPos;
    // for(int i(0); i < 4; ++i){
    //     feetPos.col(i) = getFootPos(i);
    // }
    feetPos.block(0,0,3,1) = X.block(6, 0, 3, 1);
    feetPos.block(0,1,3,1) = X.block(9, 0, 3, 1);
    feetPos.block(0,2,3,1) = X.block(12, 0, 3, 1);
    feetPos.block(0,3,3,1) = X.block(15, 0, 3, 1);
    return feetPos;
}
Eigen::Matrix<double, 3, 4> Estimator::getFeetVel(){
    Eigen::Matrix<double, 3, 4> feetVel = GetFeetSpeed2BODY(*_lowstate, FrameType::GLOBAL);
    for(int i(0); i < 4; ++i){
        feetVel.col(i) += getVelocity();
    }
    return feetVel;
}
Eigen::Matrix<double, 3, 4> Estimator::getPosFeet2BGlobal(){
    Eigen::Matrix<double, 3, 4> feet2BPos;
    for(int i(0); i < 4; ++i){
         feet2BPos.col(i) = getFootPos(i) - getPosition();
    }
    return feet2BPos;
}

LowState* Estimator::getLowState(){
    return this->_lowstate;
}

Eigen::Vector3d Estimator::getPcom(){
    return _pcom;
}

Eigen::Vector3d Estimator::getVcom(){
    return _vcom;
}