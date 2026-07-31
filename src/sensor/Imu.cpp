#include"sensor/Imu.h"

Imu::Imu(){
}

void Imu::Imu_Initial(){
    this->quaternion[0] = 1;
    this->quaternion[1] = 0;
    this->quaternion[2] = 0;
    this->quaternion[3] = 0;
}

void Imu::Imu_Update(){

}

Eigen::Matrix<double,3,3> Imu::GetRotMat(){
    Eigen::Matrix<double, 4, 1> quat;
    quat << quaternion[0],quaternion[1],quaternion[2],quaternion[3];
    return Quat2RotMat(quat);
}

Eigen::Matrix<double,3,1> Imu::GetAcc(){
    Eigen::Matrix<double,3,1> a;
    a<< accelerometer[0],accelerometer[1],accelerometer[2];
    return a;
}

Eigen::Matrix<double,3,1> Imu::GetGyro(){
    Eigen::Matrix<double,3,1> gryo;
    gryo<< gyroscope[0],gyroscope[1],gyroscope[2];
    return gryo;
}

Eigen::Matrix<double,4,1> Imu::GetQuat(){
    Eigen::Matrix<double,4,1> q;
    q<< quaternion[0],quaternion[1],quaternion[2],quaternion[3];
    return q;
}
Eigen::Matrix<double,3,1> Imu::getAccGlobal(){
        return GetRotMat() * GetAcc();
}

Eigen::Matrix<double,3,1> Imu::getGyroGlobal(){
    return GetRotMat() * GetGyro();
}

double Imu::getYaw(){
    return rotMatToRPY(GetRotMat())(2);
}

double Imu::getDYaw(){
    return getGyroGlobal()(2);
}

double Imu::getRoll(){
    double roll=0;
    double w = quaternion[0];
    double x = quaternion[1];
    double y = quaternion[2];
    double z = quaternion[3];

    roll = atan2(2*(w*x + y*z),1-2*(x*x+y*y));
    return roll;
}
double Imu::getPitch(){
    double pitch=0;
    double w = quaternion[0];
    double x = quaternion[1];
    double y = quaternion[2];
    double z = quaternion[3];

    pitch = asin(2*(w*y-z*x)) ;
    return pitch;
}
Imu::~Imu(){
    // _serial.close();
}
