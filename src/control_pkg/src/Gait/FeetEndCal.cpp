#include "Gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(Estimator* est,LowState* lowstate,WaveGenerator* wave,CTP* ctp,Eigen::Matrix<int,4,1>& contact)
: _est(est), _lowState(lowstate)
,_tsw(wave->getTsw()),_footpos_Global(wave->_FootPos)
,A(ctp->getA_addr()),_contact(contact)
,dfooth(0.04){
    _Tstance  = wave->getTstance();
    _Tswing   = wave->getTswing();

    kp = 0.15;
    off_set = -0.01;
    SymPb1 << _length_ + off_set,-_weigh_-_labad_,0;
    SymPb2 << _length_ + off_set, _weigh_+_labad_,0;
    SymPb3 << -_length_ + off_set,-_weigh_-_labad_,0;
    SymPb4 << -_length_ + off_set, _weigh_ + _labad_,0;
    SymPb.push_back(SymPb1);
    SymPb.push_back(SymPb2);
    SymPb.push_back(SymPb3);
    SymPb.push_back(SymPb4);

    Pcomtouch.resize(4);
    Psymtouch.resize(4);
    P1.resize(4);
    P2.resize(4);
    P3.resize(4);
    P4.resize(4);
    Pswend.resize(4);
    Pstend.resize(4);
    Pstsw.resize(4);
}

void FeetEndCal::cal(Eigen::Matrix<double,3,4>& FootdesirePos,Eigen::Matrix<double,3,4>& FootdesireVelocity,Eigen::Vector3d _dvO,double _dwz){
    _yaw = _lowState->_imu.getYaw();
    Eigen::Vector3f _wb = _lowState->_imu.GetGyro();
    Eigen::Vector3d pcom = _est->getPcom();
    Eigen::Vector3d vcom = _est->getVcom();
    // std::cout<<"pcom \n"<< pcom<<std::endl;
    // std::cout<<"vcom \n"<< vcom<<std::endl;
    // std::cout<<"_yaw \n"<< _yaw<<std::endl;
    // std::cout<<"_wb \n"<< _wb<<std::endl;

    for(int i(0);i<4;++i){
        // 落脚点计算：
        Pcomtouch[i] = pcom + _dvO*((1 - _tsw[i]) * _Tswing);
        FaiZtouch[i] = _yaw + _dwz * ((1 - _tsw[i]) * _Tswing);

        Psymtouch[i] = Pcomtouch[i] + rotz(FaiZtouch[i]).cast<double>() * SymPb[i];
        P1[i] = (_dvO * _Tswing) / 2.0f;
        P2[i] = rotz(FaiZtouch[i]).cast<double>()*(rotz(_dwz*_Tswing/2.0f).cast<double>() * SymPb[i] - SymPb[i]);
        P3[i] = kp * (vcom - _dvO);
        P4[i] = (pcom[2] / 9.81f)*(vcom.cross(_lowState->_imu.GetRotMat().cast<double>() *_wb.cast<double>()));

        // 最终的落足点坐标
        Pswend[i] = Psymtouch[i] + P1[i] + P2[i] + P3[i] + P4[i];
        Pswend[i][2] = (*A)[0] + (*A)[1] * (Pswend[i][0]) + (*A)[2] * (Pswend[i][1]);

        // 轨迹规划:
        Pstend[i] = _footpos_Global.col(i);
        Pstsw[i] = Pswend[i] - Pstend[i];
        if(_contact(i)==0){
            FootdesirePos(0,i) = Pstend[i][0] + Pstsw[i][0] * (3.0f * pow(_tsw[i], 2) - 2.0f * pow(_tsw[i], 3));
            FootdesirePos(1,i) = Pstend[i][1] + Pstsw[i][1] * (3.0f * pow(_tsw[i], 2) - 2.0f * pow(_tsw[i], 3));
            FootdesireVelocity(0,i) = Pstsw[i][0] * (6.0f * _tsw[i] - 6.0f * pow(_tsw[i], 2)) / _Tswing;
            FootdesireVelocity(1,i) = Pstsw[i][1] * (6.0f * _tsw[i] - 6.0f * pow(_tsw[i], 2)) / _Tswing;

            double h_start = Pstend[i][2];
            double h_end = Pswend[i][2];
            FootdesirePos(2,i) = h_start + (h_end - h_start) * _tsw[i] + dfooth * sin(M_PI * _tsw[i]);
          // 速度：严格遵守微积分链式法则求导
          FootdesireVelocity(2,i) = ((h_end - h_start) + dfooth * M_PI * cos(M_PI * _tsw[i])) / _Tswing;
        }
        else{
            FootdesireVelocity.col(i).setZero();
        }
        
    }
    _FootdesirePos = FootdesirePos;
    _FootdesireVelocity = FootdesireVelocity;
    // std::cout<<"_footPos:\n"<< _footPos<<std::endl;
}
