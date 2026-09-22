#ifndef USER_CMD_H
#define USER_CMD_H

#include <atomic>
#include <pthread.h>
#include <iostream>  
#include <unistd.h> 
#include "FSM/EnumClassList.h"
#include <termios.h>
#include "sensor/Imu.h"
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>

#include <lcm/lcm-cpp.hpp>
#include "lcm_msg_cpp/lcm_vel_cmd.hpp"

#define JOYTRICK_DEVICE "/dev/input/event1"
#define JOYTRICK_Y 288
#define JOYTRICK_B 289
#define JOYTRICK_A 290
#define JOYTRICK_X 291
#define JOYTRICK_R1 295
#define JOYTRICK_R2 293
#define JOYTRICK_R3 299
#define JOYTRICK_L1 294
#define JOYTRICK_L2 292
#define JOYTRICK_L3 298

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
    Imu*                 _imu;
    int                 _state=4;
    int                 _turn = 0 ;
    float               _vx = 0;
    float               _vy = 0;
    float               _wz = 0;
    float               _lx = 0;
    float               _ly = 0;
    float               _ry = 0;
    float               _rx = 0;
private:
    std::atomic<bool>  _running{true};
    float normalizeAxis(int value, const input_absinfo& info);
    float deadZone(float value, float zone = 0.08f);
    lcm::LCM *          _lcm;
    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcm_msg::lcm_vel_cmd* msg);
    int _joytrick_fd;
    input_absinfo axisInfo[ABS_CNT]{};
    float _leftX = 0.0f;
    float _leftY = 0.0f;
    float _rightX = 0.0f;
    float _rightY = 0.0f;
};
#endif
