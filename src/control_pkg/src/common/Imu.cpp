#include"Imu.h"

Imu::Imu()
{
    Imu_Initial();
}
void Imu::Imu_Initial(){
    this->quaternion[0] = 1;
    this->quaternion[1] = 0;
    this->quaternion[2] = 0;
    this->quaternion[3] = 0;
    this->gyroscope[0] = 0;
    this->gyroscope[1] = 0;
    this->gyroscope[2] = 0;
    this->accelerometer[0] = 0;
    this->accelerometer[1] = 0;
    this->accelerometer[2] = 9.81;
}
void Imu::Imu_Update(){
}

Eigen::Matrix<double,3,3> Imu::GetRotMat(){
    Eigen::Matrix<double, 4, 1> quat;
    quat << (double)quaternion[0],(double)quaternion[1],(double)quaternion[2],(double)quaternion[3];
    return Quat2RotMat(quat);
}

Eigen::Matrix<double,3,1> Imu::GetAcc(){
    Eigen::Matrix<double,3,1> a;
    a<< (double)accelerometer[0],(double)accelerometer[1],(double)accelerometer[2];
    return a;
}

Eigen::Matrix<double,3,1> Imu::GetGyro(){
    Eigen::Matrix<double,3,1> gryo;
    gryo<< (double)gyroscope[0],(double)gyroscope[1],(double)gyroscope[2];
    return gryo;
}

Eigen::Matrix<double,4,1> Imu::GetQuat(){
    Eigen::Matrix<double,4,1> q;
    q<< (double)quaternion[0],(double)quaternion[1],(double)quaternion[2],(double)quaternion[3];
    return q;
}
Eigen::Matrix<double,3,1> Imu::getAccGlobal(){
        return GetRotMat() * GetAcc();
}

Eigen::Matrix<double,3,1> Imu::getGyroGlobal(){
    return GetRotMat() * GetGyro();
}

float Imu::getYaw(){
    return rotMatToRPY(GetRotMat())(2);
}

float Imu::getDYaw(){
    return getGyroGlobal()(2);
}

float Imu::getRoll(){
    float roll=0;
    float w = quaternion[0];
    float x = quaternion[1];
    float y = quaternion[2];
    float z = quaternion[3];

    roll = atan2(2*(w*x + y*z),1-2*(x*x+y*y));
    return roll;
}
float Imu::getPitch(){
    float pitch=0;
    float w = quaternion[0];
    float x = quaternion[1];
    float y = quaternion[2];
    float z = quaternion[3];

    pitch = asin(2*(w*y-z*x)) ;
    return pitch;
}
Imu::~Imu(){
    // _serial.close();
}
