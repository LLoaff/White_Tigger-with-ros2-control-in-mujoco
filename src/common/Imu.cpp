#include"Imu.h"


Imu::Imu()
{
}
void Imu::Imu_Initial(){
    this->quaternion[0] = 1;
    this->quaternion[1] = 0;
    this->quaternion[2] = 0;
    this->quaternion[3] = 0;
}
void Imu::Imu_Update(){
    // std::cout<<"quan 0: "<<quaternion[0]<<" quan 1: "<<quaternion[1]<<" quan 2: "<<quaternion[2]<<" quan 3: "<<quaternion[3]<<std::endl;
    // if(_serial.isOpen()){
    //     uint8_t send_data=0x0a;
    //     _serial.writeData(&send_data,1);
    
    //     if(_serial.getReadBufferUsedLen()>0){
    //         _serial.readData(data_buff,42);
    //         if(data_buff[0]==0x0a && data_buff[41]==0x0b){
    //             memcpy(&quaternion[0], &data_buff[1], 4);
    //             memcpy(&quaternion[1], &data_buff[5], 4);
    //             memcpy(&quaternion[2], &data_buff[9], 4);
    //             memcpy(&quaternion[3], &data_buff[13], 4);

    //             memcpy(&accelerometer[0], &data_buff[17], 4);
    //             memcpy(&accelerometer[1], &data_buff[21], 4);
    //             memcpy(&accelerometer[2], &data_buff[25], 4);

    //             memcpy(&gyroscope[0], &data_buff[29], 4);
    //             memcpy(&gyroscope[1], &data_buff[33], 4);
    //             memcpy(&gyroscope[2], &data_buff[37], 4);
    //             #ifdef LOWSTATE_DEBUG
    //             syslog(LOG_INFO,"imu: q:%.3f %.3f %.3f %.3f - ac:%.3f %.3f %.3f - gy:%.3f %.3f %.3f "
    //                 ,quaternion[0],quaternion[1],quaternion[2],quaternion[3],accelerometer[0],accelerometer[1],accelerometer[2],gyroscope[0],gyroscope[1],gyroscope[2]);
    //             #endif
    //         }
    //     }
    // }
    
}

Eigen::Matrix<float,3,3> Imu::GetRotMat(){
    Eigen::Matrix<float, 4, 1> quat;
    quat << quaternion[0],quaternion[1],quaternion[2],quaternion[3];
    return Quat2RotMat(quat);
}

Eigen::Matrix<float,3,1> Imu::GetAcc(){
    Eigen::Matrix<float,3,1> a;
    a<< accelerometer[0],accelerometer[1],accelerometer[2];
    return a;
}

Eigen::Matrix<float,3,1> Imu::GetGyro(){
    Eigen::Matrix<float,3,1> gryo;
    gryo<< gyroscope[0],gyroscope[1],gyroscope[2];
    return gryo;
}

Eigen::Matrix<float,4,1> Imu::GetQuat(){
    Eigen::Matrix<float,4,1> q;
    q<< quaternion[0],quaternion[1],quaternion[2],quaternion[3];
    return q;
}
Eigen::Matrix<float,3,1> Imu::getAccGlobal(){
        return GetRotMat() * GetAcc();
}

Eigen::Matrix<float,3,1> Imu::getGyroGlobal(){
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
