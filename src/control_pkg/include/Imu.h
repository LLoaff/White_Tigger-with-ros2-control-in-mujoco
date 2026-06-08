#ifndef IMU_H
#define IMU_H

#include "CSerialPort/SerialPort.h"
#include <iostream>  
#include <unistd.h> 
#include "eigen3/Eigen/Dense"
#include "mathtool.h"
#include <syslog.h>

class Imu
{
public:
    Imu();
    ~Imu();
    void Imu_Initial();
    void Imu_Update();
    Eigen::Matrix<float,3,3> GetRotMat();
    Eigen::Matrix<float,3,1> GetAcc();
    Eigen::Matrix<float,3,1> GetGyro();
    Eigen::Matrix<float,4,1> GetQuat();
    Eigen::Matrix<float,3,1> getAccGlobal();
    Eigen::Matrix<float,3,1> getGyroGlobal();
    float getPitch();
    float getRoll();
    float getYaw();
    float getDYaw();

    float quaternion[4];    // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];
private:
    itas109::CSerialPort _serial;

    uint8_t data_buff[42];


};

#endif