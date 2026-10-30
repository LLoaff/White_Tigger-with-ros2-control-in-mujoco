#ifndef IMU_H
#define IMU_H

#include <iostream>  
#include <unistd.h> 
#include "eigen3/Eigen/Dense"
#include "math/mathtool.h"
#include <syslog.h>
#include "math/TimeMaker.h"
#include <iomanip>
#ifdef USE_SIM
#include <mujoco/mujoco.h>

class Imu
{
public:
    Imu(mjModel *model, mjData *data);
    ~Imu();
    void Imu_Initial();
    void Imu_Update();
    void Imu_GetBias();
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
    // uint8_t data_buff[42];
    int _imu_acc_id =  0;
    int _imu_gyro_id = 0;
    int _imu_quat_id = 0;

    float bias_accx = 0;
    float bias_accy = 0;
    float bias_accz = 0;
    float bias_gyrx = 0;
    float bias_gyry = 0;
    float bias_gyrz = 0;

    int counts_acc = 0;
    int counts_gyr = 0;
    mjModel*  _model;
    mjData*   _data;
};
#else
#include "CSerialPort/SerialPort.h"

class Imu
{
public:
    Imu();
    ~Imu();
    void Imu_Initial();
    void Imu_Update();
    void Imu_GetBias();

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

    float bias_accx = 0;
    float bias_accy = 0;
    float bias_accz = 0;
    float bias_gyrx = 0;
    float bias_gyry = 0;
    float bias_gyrz = 0;

    int counts_acc = 0;
    int counts_gyr = 0;
    itas109::CSerialPort _serial;
    std::vector<uint8_t> recv_buffer;

};
#endif
#endif