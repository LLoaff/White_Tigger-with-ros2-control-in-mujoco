#ifndef IMU_H
#define IMU_H

#include <iostream>  
#include <unistd.h> 
#include "eigen3/Eigen/Dense"
#include "math/mathtool.h"
#include <syslog.h>
#include <mujoco/mujoco.h>

class Imu
{
public:
    Imu(mjModel *model, mjData *data);
    ~Imu();
    void Imu_Initial();
    void Imu_Update();
    Eigen::Matrix<double,3,3> GetRotMat();
    Eigen::Matrix<double,3,1> GetAcc();
    Eigen::Matrix<double,3,1> GetGyro();
    Eigen::Matrix<double,4,1> GetQuat();
    Eigen::Matrix<double,3,1> getAccGlobal();
    Eigen::Matrix<double,3,1> getGyroGlobal();
    double getPitch();
    double getRoll();
    double getYaw();
    double getDYaw();

    double quaternion[4];    // w, x, y, z
    double gyroscope[3];
    double accelerometer[3];
private:
    // uint8_t data_buff[42];
    int _imu_acc_id = 0;
    int _imu_gyro_id = 0;
    int _imu_quat_id = 0;
    mjModel*  _model;
    mjData*   _data;
};

#endif