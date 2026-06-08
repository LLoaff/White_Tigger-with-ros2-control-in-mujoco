#ifndef USER_CMD_H
#define USER_CMD_H

#include <pthread.h>
#include <iostream>  
#include <unistd.h> 
#include "CSerialPort/SerialPort.h"
#include "EnumClassList.h"
#include <termios.h>
#include "Imu.h"


#include <lcm/lcm-cpp.hpp>
#include "lcm_msg_cpp/lcm_vel_cmd.hpp"
typedef struct __packed{
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint8_t s1;
        uint8_t s2;
}RC_Ctl_t;

class UserCmd
{
public:
    UserCmd( );
    ~UserCmd();
    UserValue GetUserValue();
    static void *KeyBoardInit(void * arg);
    void KeyBoardGet();
    pthread_t           _thread;
    UserValue           _user_value = UserValue::PASSIVE;
    itas109::CSerialPort _serial;
    uint8_t             _srerial_data[52];
    RC_Ctl_t            R_Data;
    Imu*                 _imu;
    int                 _state;
    int                 _turn;
    float               _vx;
    float               _vy;
    float               _wz;
private:
    lcm::LCM *          _lcm;
    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcm_msg::lcm_vel_cmd* msg);


};
#endif