#include "FSM/UserCmd.h"

UserCmd::UserCmd()
{
    // _serial.init("/dev/ttyUSB0",
    //     460800 ,
    //     itas109::ParityNone,
    //     itas109::DataBits8,
    //     itas109::StopOne);
    //     _serial.open();
        
    //     if(!_serial.isOpen()){
    //         printf("open fail--------- reason\n");
    //         _serial.close();
    //     }
        std::cout<<"User Open Success!!!"<<std::endl;

        _lcm = new lcm::LCM();
        if(!_lcm->good())        {
            std::cerr<<"LCM init failed!"<<std::endl;
            exit(-1);
        }
        _lcm->subscribe("cmd_vel", &UserCmd::handleMessage, this);

         pthread_create(&_thread,NULL,UserCmd::KeyBoardInit,this);
}

void* UserCmd::KeyBoardInit(void * arg)
{
    UserCmd * user = static_cast<UserCmd *>(arg);
    user ->KeyBoardGet();
    return NULL;
}

void UserCmd::KeyBoardGet()
{
    char key=0;
        while(true)
        {
            
            // key = getchar();
            // switch (key)
            // {
            // case 'p':
            //     _user_value = UserValue::PASSIVE;
            //     break;
            // case 's':
            //     _user_value = UserValue::STAND;
            //     break;
            // case 'f':
            //     _user_value = UserValue::FREE;
            //     break;
            // // case 'b':
            // //     pthread_mutex_lock(&_mutex);
            // //     _user_value = UserValue::BALANCE;
            // //     pthread_mutex_unlock(&_mutex);
            // //     break;
            // case 't':
            //     _user_value = UserValue::TROTTING;
            //     break;
            // default:
            //     break;
            // }

            // if(_serial.getReadBufferUsedLen()>=18){

            //     _serial.readData(_srerial_data,18);

            //     R_Data.ch0 = ((int16_t)_srerial_data[0] | ((int16_t)_srerial_data[1] << 8)) & 0x07FF;
            //     R_Data.ch1 = (((int16_t)_srerial_data[1] >> 3) | ((int16_t)_srerial_data[2] << 5))& 0x07FF;
            //     R_Data.ch2 = (((int16_t)_srerial_data[2] >> 6) | ((int16_t)_srerial_data[3] << 2) |((int16_t)_srerial_data[4] << 10)) & 0x07FF;
            //     R_Data.ch3 = (((int16_t)_srerial_data[4] >> 1) | ((int16_t)_srerial_data[5]<<7)) &0x07FF;
            //     R_Data.s1 = ((_srerial_data[5] >> 4) & 0x000C) >> 2;
            //     R_Data.s2 = ((_srerial_data[5] >> 4) & 0x0003);

                
            // }

            // if(_serial.isOpen()){
                
            //         _serial.readData(_srerial_data,12);
            //         //     printf("******\n");

            //         // for(int i=0;i<52;i++){
            //         //     printf("%02X ", (unsigned char)_srerial_data[i]);
                        
            //         // }   
            //         //     printf("******\n");

            //         if(_srerial_data[0] == 0x0a && _srerial_data[11] == 0x0b){
            //             // memcpy(&_imu->quaternion[0], &_srerial_data[1], 4);
            //             // memcpy(&_imu->quaternion[1], &_srerial_data[5], 4);
            //             // memcpy(&_imu->quaternion[2], &_srerial_data[9], 4);
            //             // memcpy(&_imu->quaternion[3], &_srerial_data[13], 4);

            //             // memcpy(&_imu->accelerometer[0], &_srerial_data[17], 4);
            //             // memcpy(&_imu->accelerometer[1], &_srerial_data[21], 4);
            //             // memcpy(&_imu->accelerometer[2], &_srerial_data[25], 4);

            //             // memcpy(&_imu->gyroscope[0], &_srerial_data[29], 4);
            //             // memcpy(&_imu->gyroscope[1], &_srerial_data[33], 4);
            //             // memcpy(&_imu->gyroscope[2], &_srerial_data[37], 4);
            //             R_Data.ch0 = (uint16_t)_srerial_data[1]<<8 | (uint16_t)_srerial_data[2];
            //             R_Data.ch1 = (uint16_t)_srerial_data[3]<<8 | (uint16_t)_srerial_data[4];
            //             R_Data.ch2 = (uint16_t)_srerial_data[5]<<8 | (uint16_t)_srerial_data[6];
            //             R_Data.ch3 = (uint16_t)_srerial_data[7]<<8 | (uint16_t)_srerial_data[8];
            //             std::cout<<"ch0: "<<R_Data.ch0<<" ch1: "<<R_Data.ch1<<" ch2: "<<R_Data.ch2<<" ch3: "
            //             <<R_Data.ch3<<std::endl;
            //         }
                
            //     }
            _lcm->handle();
        }
}
void UserCmd::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcm_msg::lcm_vel_cmd* msg){
    this->_state = msg->state;
    this->_vx = msg->vx;
    this->_vy = msg->vy;
    this->_wz = msg->vw;
    this->_turn = msg->turn;
    // std::cout<<"state: "<<this->_state<<" vx: "<<this->_vx<<" vy: "<<this->_vy<<" wz: "<<this->_wz<<" turn: "<<this->_turn<<std::endl;
}
UserValue UserCmd::GetUserValue()
{

    // if(R_Data.s1 == 3 && R_Data.s2 ==3){
    //     _user_value = UserValue::PASSIVE;
    // }
    // else if(R_Data.s1 == 3 && R_Data.s2 ==1){
    //     _user_value = UserValue::FREE;
    // }
    // else if(R_Data.s1 == 1 && R_Data.s2 ==3){
    //     _user_value = UserValue::STAND;
    // }
    // else if(R_Data.s1 == 1 && R_Data.s2 ==1){
    //     _user_value = UserValue::TROTTING;
    // }

    switch (this->_state)
    {
    case 0:
        _user_value = UserValue::PASSIVE;
        break;
    case 1:
        _user_value = UserValue::FREE;
        break;
    case 2:
        _user_value = UserValue::STAND;
        break;
    case 3:
        _user_value = UserValue::TROTTING;
        break;
    case 4:
        _user_value = UserValue::SIT_DOWN;
        break;
    default:
        break;
    }
    UserValue val = _user_value;
    return val;
}

UserCmd::~UserCmd()
{
    pthread_join(_thread, NULL);
    // _serial.close();
}