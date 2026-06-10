#include "MPC/CTP.h"

CTP::CTP(Estimator* _est,Eigen::Matrix<double, 3, 4>& footpos, float& _user_vx_B, float& _user_vy_B, float& _user_vz_B):
_est(_est), _footPosGlobal(footpos),_dt(MPC_T), _user_vx_B(_user_vx_B), _user_vy_B(_user_vy_B), _user_vz_B(_user_vz_B),_imu(&_est->getLowState()->_imu){
    w = 0.5;
    W = Eigen::MatrixXd(4,3);
    Z = Eigen::VectorXd(4);
    A = Eigen::VectorXd(3);
    _A = Eigen::VectorXd(3);
    N = Eigen::VectorXd(3);
    N_ = Eigen::VectorXd(3);

    // _mpc_steps 为预测步长，需要记录 _mpc_steps+1 个状态，每个状态为 13 维向量
    desirex.resize(_mpc_steps + 1, Eigen::VectorXd::Zero(13));
    A.setZero();
    _A.setZero();
    N.setZero();
    N_.setZero();
    Z.setZero();
    W.setZero();
    Tao.setZero();
    D = Eigen::MatrixXd(13 *_mpc_steps, 1);

    _d_Vcom_B.setZero();
    _d_Vcom_O.setZero();
    _d_Wcom_O.setZero();
    _Pcom_O.setZero();
    _d_Pcom_O.setZero();
    
    _d_yaw = _imu->getYaw(); // 第一次以当前yaw为期望角
}
    
void CTP::update(){
    _d_vx_B = 0.7*_d_vx_B + 0.3*(double)_user_vx_B;
    _d_vy_B = 0.7*_d_vy_B + 0.3*(double)_user_vy_B;
    _d_wz_B = 0.7*_d_wz_B + 0.3*(double)_user_vz_B;
    // std::cout<<"_d_vx_B: "<<_d_vx_B<<" _d_vy_B: "<<_d_vy_B<<" _d_wz_B: "<<_d_wz_B<<std::endl;
    Eigen::Matrix<double, 3, 3> tfz;

    for(int i(0);i<_mpc_steps; ++i){

        if(i==0){
            _d_yaw = _imu->getYaw(); // 第一次以当前yaw为期望角
            tfz = rotz(_d_yaw);

            _d_wz_O = _d_wz_B;
            _d_Wcom_O << 0,0,_d_wz_O;       // 期望偏航角速度更新

            _d_Vcom_B<<_d_vx_B, _d_vy_B, 0;
            _d_Vcom_O = tfz * _d_Vcom_B;    // 期望质心速度更新

            // 坡度估计：
            // 把Pst·end的z分离出来
            Z<<_footPosGlobal(2,0), _footPosGlobal(2,1), _footPosGlobal(2,2), _footPosGlobal(2,3);
            // 分离出Pst·end的x、y分量，组成矩阵W
            W<<1,_footPosGlobal(0,0),_footPosGlobal(1,0),
               1,_footPosGlobal(0,1),_footPosGlobal(1,1),
               1,_footPosGlobal(0,2),_footPosGlobal(1,2),
               1,_footPosGlobal(0,3),_footPosGlobal(1,3);

            Eigen::Matrix3d WtW = W.transpose() * W;
            double det = WtW.determinant();
            if (det > 1e-8f)
            {
                Eigen::Vector3d A_new = WtW.inverse() * W.transpose() * Z;
                A = 0.2f * A_new + 0.8f * _A;
            }
            else
            {
                A = _A; // 保留上一帧，不更新
            }
            _A = A;
            // 地面法向量n赋值
            N<<-A(1), -A(2), 1;
            // 归一化的法向量
            N_ = N * (1.0f/ pow(pow(N(0),2)+pow(N(1),2)+1,0.5));
            Tao = tfz.inverse() * N_; //  求矩阵Tao
            // _d_pitch = std::asin(Tao(1));
            // _d_roll = std::atan(Tao(0)/Tao(2));
            _d_pitch = _imu->getPitch();
            _d_roll = _imu->getRoll();

            _Pcom_O = _est->getPcom();
            _d_Pcom_O = _Pcom_O; // 期望位置更新

            desirex[i]<< _d_roll,_d_pitch,_d_yaw,_Pcom_O,_imu->GetRotMat().cast<double>()*_d_Wcom_O,_d_Vcom_O,-9.81;
        }
        _d_yaw = _d_yaw + _d_wz_O * _dt; // 期望yaw更新
        tfz = rotz(_d_yaw);
        _d_Vcom_O = tfz * _d_Vcom_B;    // 期望质心速度更新
        // std::cout<<"_d_Vcom_O: \n"<<_d_Vcom_O<<std::endl;
        _d_Pcom_O = _d_Pcom_O + _d_Vcom_O * _dt; // 期望质心位置更新
        // std::cout<<"_d_Pcom_O: \n"<<_d_Pcom_O<<std::endl;

        _d_Pcom_O(2) = _d_high;

        Tao = tfz.inverse() * N_; //  求矩阵Tao
        _d_pitch = std::asin(Tao(1));
        _d_roll = std::atan(Tao(0)/Tao(2));

        desirex[i+1]<< _d_roll,_d_pitch,_d_yaw,_d_Pcom_O,_imu->GetRotMat().cast<double>()*_d_Wcom_O,_d_Vcom_O,-9.81;
    }
    for(int i(0);i<_mpc_steps; ++i){
        D.block(13*i,0,13,1) = desirex[i+1];
        //  std::cout<<"desirex: i \n"<<i+1<<" \n "<<desirex[i+1]<<std::endl;

    }
    // std::cout<<"desirex: \n"<<desirex[1]<<std::endl;
}

Eigen::VectorXd * CTP::getA_addr(){
    return &A;
}


