#include "FSM/UserCmd.h"

UserCmd::UserCmd(){
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

void* UserCmd::KeyBoardInit(void * arg){
    UserCmd * user = static_cast<UserCmd *>(arg);
    user ->KeyBoardGet();
    return NULL;
}

void UserCmd::KeyBoardGet(){
        while(true){
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

UserValue UserCmd::GetUserValue(){
    switch (this->_state){
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

UserCmd::~UserCmd(){
    pthread_join(_thread, NULL);
    // _serial.close();
}