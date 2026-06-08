#ifndef LOWCMD_H
#define LOWCMD_H

#include <unistd.h>
#include "LowState.h"
#include "dm_SDK/damiao.h"
#include "dm_SDK/SerialPort.h"



class LowCmd
{
    public:
        LowCmd();
        void Update();
        void SetQ(Eigen::Matrix<float,12,1> q);
        void SetQ(int id,float q);
        void SetQ(int leg_id , Eigen::Matrix<float,3,1> q);
        void SetDq(int leg_id,Eigen::Matrix<float,3,1> dq);
        void SetDq(Eigen::Matrix<float,12,1> dq);
        void SetP(int leg_id,Eigen::Matrix<float,3,1> p);
        void SetP(Eigen::Matrix<float,12,1> p);
        void SetZeroP();
        void SetD(int leg_id,Eigen::Matrix<float,3,1> d);
        void SetD(Eigen::Matrix<float,12,1> d);
        void SetZeroD();
        void SetTau(Eigen::Matrix<float,12,1> tau);
        void SetTau(int leg_id,Eigen::Matrix<float,3,1> tau);
        void SetZeroTau(int legID);
        void SetZeroTau();
        void SetZeroDq(int legID);
        void SetZeroDq();
        void SetFree();
        
        void setStableGain(int legID);
        void setStableGain();
        void setSwingGain(int legID);

        Eigen::Matrix<float,3,4> getQ();
        Eigen::Matrix<float,12,1> getQ12();
        Eigen::Matrix<float,12,1> getW12();
        Eigen::Matrix<float,12,1> getInitialQ12();
        ~LowCmd();
        std::shared_ptr<SerialPort> serial;
        damiao::Motor_Control _motor_cmd;
        LowState _state;        // LowState类里 的数据是数组
        private:
            float start_angle[12]; // 存储开机时电机初始角度  弧度制
            struct Motor_State _cmd[12];
            

};


#endif