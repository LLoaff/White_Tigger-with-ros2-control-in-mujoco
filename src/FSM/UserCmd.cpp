#include "FSM/UserCmd.h"

UserCmd::UserCmd(){
    _lcm = new lcm::LCM();

    _joytrick_fd = open(JOYTRICK_DEVICE, O_RDONLY | O_CLOEXEC);
    if(_joytrick_fd<0){
        std::perror("无法打开手柄");
    }else{
        
        for(int axis=0;axis<ABS_CNT;axis++){
            ioctl(_joytrick_fd,EVIOCGABS(axis),&axisInfo[axis]);
        }
    }
    
    if(!_lcm->good())        {
        std::cerr<<"LCM init failed!"<<std::endl;
        exit(-1);
    }
    _lcm->subscribe("cmd_vel", &UserCmd::handleMessage, this);
    std::cout<<"User Open Success!!!"<<std::endl;

    pthread_create(&_thread,NULL,UserCmd::KeyBoardInit,this);
}

void* UserCmd::KeyBoardInit(void * arg){
    UserCmd * user = static_cast<UserCmd *>(arg);
    user ->KeyBoardGet();
    return NULL;
}

void UserCmd::KeyBoardGet(){
    while(_running){
        pollfd pfd {_joytrick_fd, POLLIN, 0};
        int ready = poll(&pfd,1,-1);
        if(ready<0){
            if(errno==EINTR) continue;
            std::perror("joytrick poll error");
            break;
        }

        input_event ev{};
        ssize_t n = read(_joytrick_fd,&ev,sizeof(ev));
        if(n<0){
            if(errno==EINTR) continue;
            std::perror("joytrick read error");
            break;
        }
        if(n!=sizeof(ev)){
            continue;
        }
        if(ev.type == EV_KEY){
            switch(ev.code){
                case JOYTRICK_Y:
                    if(ev.value==1){
                        _user_value = UserValue::TROTTING;
                    }
                    break;
                case JOYTRICK_B:
                    if(ev.value==1){
                        _user_value = UserValue::BALANCE;
                    }
                    break;
                case JOYTRICK_A:
                    if(ev.value==1){
                        _user_value = UserValue::SIT_DOWN;
                    }
                    break;
                case JOYTRICK_X:
                    if(ev.value==1){
                        _user_value = UserValue::STAND ;
                    }
                    break;
                case JOYTRICK_R1:
                    if(ev.value==1){
                        _user_value = UserValue::PASSIVE;
                    }
                    break;
                case JOYTRICK_R2:
                    if(ev.value==1){
                        _user_value = UserValue::PASSIVE;
                    }
                    break;
                    case JOYTRICK_R3:
                    if(ev.value==1){
                        // _user_value = UserValue::SIT_DOWN;
                    }
                    break;
                    case JOYTRICK_L1:
                    if(ev.value==1){
                        _user_value = UserValue::PASSIVE;
                    }
                    break;
                    case JOYTRICK_L2:
                    if(ev.value==1){
                        _user_value = UserValue::PASSIVE;
                    }
                    break;
                    case JOYTRICK_L3:
                    if(ev.value==1){
                        // _user_value = UserValue::SIT_DOWN;
                    }
                    break;
                default:
                    break;
            }
        }
        if(ev.type == EV_ABS){
            if(ev.code == ABS_HAT0X || ev.code == ABS_HAT0Y){
                // 十字键盘 处理
                continue;
            }
            float value = -deadZone(normalizeAxis(ev.value,axisInfo[ev.code]));
            switch(ev.code){
                case ABS_X://左x
                    _lx = value;
                    break;
                case ABS_Y://左y
                    _ly = value;
                    break;
                case ABS_Z: //右x
                    _rx = value;
                    break;
                case ABS_RZ://右y
                    _ry = value;
                    break;
                default:
                    break;
            }
            // std::cout<<"lx: "<<_lx<<" ly: "<<_ly<<" rx: "<<_rx<<" ry: "<<_ry<<std::endl;
        }
        // _lcm->handleTimeout(1);
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
    // switch (this->_state){
    // case 0:
    //     _user_value = UserValue::PASSIVE;
    //     break;
    // case 1:
    //     _user_value = UserValue::FREE;
    //     break;
    // case 2:
    //     _user_value = UserValue::STAND;
    //     break;
    // case 3:
    //     _user_value = UserValue::TROTTING;
    //     break;
    // case 4:
    //     _user_value = UserValue::SIT_DOWN;
    //     break;
    // case 5:
    //     _user_value = UserValue::TROTTING_MPC;
    //     break;
    // default:
    //     break;
    // }
    UserValue val = _user_value;
    return val;
}
float UserCmd::normalizeAxis(int value, const input_absinfo& info) {
    const float center = (info.minimum + info.maximum) * 0.5f;
    if (value >= center) {
        return (value - center) / (info.maximum - center);
    }
    return (value - center) / (center - info.minimum);
}
float UserCmd::deadZone(float value, float zone) {
    if (std::fabs(value) < zone) {
        return 0.0f;
    }

    // 去掉死区后重新拉伸到 -1.0 ~ 1.0
    return (value > 0)
        ? (value - zone) / (1.0f - zone)
        : (value + zone) / (1.0f - zone);
}
UserCmd::~UserCmd(){
    _running = false;
    pthread_join(_thread, NULL);
    delete _lcm;
}
