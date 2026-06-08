#include "Kenimatics_normal_solution.h"

                            // group用来区别是第几条腿
Eigen::Matrix<float,3,1> GetPos_H(uint8_t group, float theta1, float theta2, float theta3)
{
    Eigen::Matrix<float,3,1> matrix;
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;
    
    if(group == 0 || group == 2)
        l1 = -l1;

    matrix(0) = l3 * sin(theta2+theta3) + l2 * sin(theta2) ;
    matrix(1) = -l3 * sin(theta1)*cos(theta2 + theta3) + l1 * cos(theta1) - l2 * cos(theta2) * sin(theta1);
    matrix(2) = l3 * cos(theta1) * cos(theta2+theta3) + l1 * sin(theta1) + l2* cos(theta1) * cos(theta2);

    return matrix;
}

Eigen::Matrix<float,3,1> GetPos_B(uint8_t group, float theta1, float theta2, float theta3)
{
    Eigen::Matrix<float,3,1> matrix;
    float length = _length_;
    float weigh = _weigh_;
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;
    
    if(group == 0 || group == 2){
        l1 = -l1;
    }
    
    if(group == 0){
        weigh = -weigh;
    }
    else if(group == 3){
        length = -length;
    }
    else if(group == 2){
        weigh = -weigh;
        length = -length;
    }

    matrix(0) = l3 * sin(theta2+theta3) + l2 * sin(theta2) +length;
    matrix(1) = -l3 * sin(theta1)*cos(theta2 + theta3) + l1 * cos(theta1) - l2 * cos(theta2) * sin(theta1)+weigh;
    matrix(2) = l3 * cos(theta1) * cos(theta2+theta3) + l1 * sin(theta1) + l2* cos(theta1) * cos(theta2);

    return matrix;
}

Eigen::Matrix<float,3,1> GetPos_S(Eigen::Matrix<float,3,1> init_pos,uint8_t id,float theta1, float theta2, float theta3)
{
    Eigen::Matrix<float,3,1> matrix_s;

    matrix_s = GetPos_B(id,theta1,theta2,theta3) - init_pos;

    return matrix_s;
}

Eigen::Matrix<float,3,1> Pos_Speed(int group,float theta1, float theta2, float theta3 , float th1_s , float th2_s , float th3_s)
{
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;

    if(group == 0 || group == 2)
        l1 = -l1;

    Eigen::Matrix<float,3,1> w;
    Eigen::Matrix<float,3,1> pos_s;
    Eigen::Matrix<float,3,3> Jac;

    Jac<< 0, 
          l3*cos(theta2+theta3)+l2*cos(theta2), 
          l3*cos(theta2+theta3),

          -l1*sin(theta1)-l2*cos(theta1)*cos(theta2)-l3*cos(theta1)*cos(theta2+theta3),
          l2*sin(theta1)*sin(theta2)+l3*sin(theta1)*sin(theta2+theta3),
          l3*sin(theta1)*sin(theta2+theta3),

          l1*cos(theta1)-l2*sin(theta1)*cos(theta2)-l3*sin(theta1)*cos(theta2+theta3),
          -l2*cos(theta1)*sin(theta2)-l3*cos(theta1)*sin(theta2+theta3),
          -l3*cos(theta1)*sin(theta2+theta3);
    w<< th1_s,th2_s,th3_s;
    pos_s = Jac * w;
    // syslog(LOG_INFO, "J11=%.6f, J12=%.6f, J13=%.6f", Jac(0,0), Jac(0,1), Jac(0,2));
    // syslog(LOG_INFO, "J21=%.6f, J22=%.6f, J23=%.6f", Jac(1,0), Jac(1,1), Jac(1,2));
    // syslog(LOG_INFO, "J31=%.6f, J32=%.6f, J33=%.6f", Jac(2,0), Jac(2,1), Jac(2,2));
    // syslog(LOG_INFO, "角速度向量：w1=%.6f, w2=%.6f, w3=%.6f", th1_s, th2_s, th3_s);
    return pos_s;
}

// 计算以机身为全局坐标系的
Eigen::Matrix<double,3,4> GetFeetPos2BODY(LowState &lowstate , FrameType frame){
    Eigen::Matrix<double,3,4> feetpos;
    if(frame == FrameType::GLOBAL){
        for(int i(0);i<4;++i){  
           feetpos.col(i) = GetPos_B(i,lowstate._motor_state[3*i+0].q,lowstate._motor_state[3*i+1].q,lowstate._motor_state[3*i+2].q).cast<double>();
        }
        feetpos = lowstate._imu.GetRotMat().cast<double>()*feetpos;
        // std::cout<<"rotmat:"<< lowstate._imu.GetRotMat().cast<double>() << std::endl;
    }
    else if(frame == FrameType::BODY){
        for(int i(0);i<4;++i){  
           feetpos.col(i) = GetPos_B(i,lowstate._motor_state[3*i+0].q,lowstate._motor_state[3*i+1].q,lowstate._motor_state[3*i+2].q).cast<double>();
        }
    }
    else if(frame == FrameType::HIP){
        for(int i(0);i<4;++i){  
           feetpos.col(i) = GetPos_H(i,lowstate._motor_state[3*i+0].q,lowstate._motor_state[3*i+1].q,lowstate._motor_state[3*i+2].q).cast<double>();
        }
    }
    else {
        std::cout << "[ERROR] Frame error " << std::endl;
        exit(-1);
    }
    return feetpos;
}

Eigen::Matrix<double,3,1> GetFeetPos2BODY(LowState &lowstate,int i , FrameType frame){
    Eigen::Matrix<double,3,1> feetpos;
    if(frame == FrameType::GLOBAL){
        feetpos = GetPos_B(i,lowstate._motor_state[3*i+0].q,lowstate._motor_state[3*i+1].q,lowstate._motor_state[3*i+2].q).cast<double>();
        feetpos = lowstate._imu.GetRotMat().cast<double>()*feetpos;
        // std::cout<<"rotmat:"<< lowstate._imu.GetRotMat().cast<double>() << std::endl;
    }
    else if(frame == FrameType::BODY){
        feetpos = GetPos_B(i,lowstate._motor_state[3*i+0].q,lowstate._motor_state[3*i+1].q,lowstate._motor_state[3*i+2].q).cast<double>();
    }
    else if(frame == FrameType::HIP){
        feetpos = GetPos_H(i,lowstate._motor_state[3*i+0].q,lowstate._motor_state[3*i+1].q,lowstate._motor_state[3*i+2].q).cast<double>();
    }
    else {
        std::cout << "[ERROR] Frame error " << std::endl;
        exit(-1);
    }
    return feetpos;
}

Eigen::Matrix<double,3,4> GetFeetSpeed2BODY(LowState &lowstate , FrameType frame){
    Eigen::Matrix<double,3,4>  vel;
    for(int i(0);i<4;++i){
        vel.col(i) = Pos_Speed(i,lowstate._motor_state[3*i+0].q,lowstate._motor_state[3*i+1].q,lowstate._motor_state[3*i+2].q,
                               lowstate._motor_state[3*i+0].dq,lowstate._motor_state[3*i+1].dq,lowstate._motor_state[3*i+2].dq).cast<double>();
    }
    if(frame == FrameType::GLOBAL){
        Eigen::Matrix<double,3,4>  pos = GetFeetPos2BODY(lowstate,FrameType::GLOBAL);
        vel += skew(lowstate._imu.GetGyro().cast<double>()) * pos;     // skew是反对称矩阵，实现叉乘  速度 = 线速度+绕轴旋转角速度
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
    }
    else{
        std::cout << "[ERROR] Frame error" << std::endl;
        exit(-1);
    }
    return vel;
}
/* tau */
Eigen::Matrix<float,3,1> CalTau(
    int group,Eigen::Matrix<float,3,1> angle,Eigen::Matrix<float,3,1> w,Eigen::Matrix<float,3,3>Kp ,Eigen::Matrix<float,3,3>Kd,Eigen::Matrix<float,3,1> target_pos, Eigen::Matrix<float,3,1> target_speed,FrameType frame)
{
    Eigen::Matrix<float,3,1> f; 
    Eigen::Matrix<float,3,1> tua;
    Eigen::Matrix<float,3,1> pos_xyz; // 存储xyz
    Eigen::Matrix<float,3,1> pos_s;   // 存储xyz的速度
    Eigen::Matrix<float,3,3> Jac;
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;
    
    if(group == 0 || group == 2)
        l1 = -_labad_;
    // 计算雅可比 position
    float s1 = std::sin(angle(0));
    float s2 = std::sin(angle(1));
    float s3 = std::sin(angle(2));

    float c1 = std::cos(angle(0));
    float c2 = std::cos(angle(1));
    float c3 = std::cos(angle(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;
    Jac(0, 0) = 0;
    Jac(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    Jac(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    Jac(0, 1) = l3 * c23 + l2 * c2;
    Jac(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    Jac(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    Jac(0, 2) = l3 * c23;
    Jac(1, 2) = l3 * s1 * s23;
    Jac(2, 2) = -l3 * c1 * s23;
    pos_s = Jac * w;    // 三维速度
    if(frame == FrameType::HIP){
        pos_xyz = GetPos_H(group,angle(0),angle(1),angle(2));

    }
    else if(frame == FrameType::BODY){
        pos_xyz = GetPos_B(group,angle(0),angle(1),angle(2));
    }
    f= Kp*(target_pos-pos_xyz)+Kd*(target_speed-pos_s);
    // std::cout<< "target_pos:\n" << target_pos<<std::endl;
    // std::cout<< "id:"<< group <<"pos_xyz: \n" << pos_xyz<<std::endl;
    tua = Jac.transpose()*f;

    return tua;
}

Eigen::Matrix<float,12,1> CalTaus(
    Eigen::Matrix<float,12,1> angle,Eigen::Matrix<float,12,1> w,Eigen::Matrix<float,3,3>Kp ,Eigen::Matrix<float,3,3>Kd,Eigen::Matrix<float,12,1> target_pos, Eigen::Matrix<float,12,1> target_speed,FrameType frame)
{
    Eigen::Matrix<float,12,1> tua;
    for(int i(0);i<4;++i){
        tua.segment(3*i,3) = CalTau(i,angle.segment(3*i,3),w.segment(3*i,3),Kp,Kd,target_pos.segment(3*i,3),target_speed.segment(3*i,3),frame);
    }
    return tua;
}

// 计算一条腿的Jaco
Eigen::Matrix<double,3,3> calcJaco(int legid , Eigen::Matrix<float,3,1> q){
    Eigen::Matrix<double,3,3> Jac;
    float l1 = _labad_;
    float l2 = -_lhip_;
    float l3 = -_lknee_;
    
    if(legid == 0 || legid == 2)
        l1 = -_labad_;
    // 计算雅可比 

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;
    Jac(0, 0) = 0;
    Jac(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    Jac(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    Jac(0, 1) = l3 * c23 + l2 * c2;
    Jac(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    Jac(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    Jac(0, 2) = l3 * c23;
    Jac(1, 2) = l3 * s1 * s23;
    Jac(2, 2) = -l3 * c1 * s23;
    // std::cout<< "Jac:\n"<<Jac<<"---\n"<<std::endl;
    return Jac;
}

//计算一条腿的Tau
Eigen::Matrix<double,3,1> calcTau(int legid ,Eigen::Matrix<float,3,1> q, Eigen::Matrix<double,3,1> force){
    return calcJaco(legid,q).transpose() * force;
}

//获取所有腿的Tau
Eigen::Matrix<double,12,1> getTau(const Eigen::Matrix<float,12,1> &q, const Eigen::Matrix<double,3,4> feetForce){
    Eigen::Matrix<double,12,1> tau;
    for(int i(0); i < 4; ++i){
        tau.segment(3*i, 3) = calcTau(i,q.segment(3*i, 3), feetForce.col(i));
    }
    return tau;
}


