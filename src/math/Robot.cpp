#include "math/Robot.h"
#include "math/mathtool.h"

QuadrupedRobot::QuadrupedRobot(){
    if(use_go1_model == 1){
        _feetPosNormalStand <<0.1881,  0.1881, -0.1881, -0.1881,
                             -0.1300,  0.1300, -0.1300,  0.1300,
                             -0.3200, -0.3200, -0.3200, -0.3200;
    }else{
        _feetPosNormalStand << _length_-0.02, _length_-0.02, -_length_-0.02, -_length_-0.02,
                          -_weigh_ - _labad_ - 0.02, _weigh_ + _labad_+0.02, -_weigh_ - _labad_, _weigh_ + _labad_,
                          -0.21, -0.21, -0.21, -0.21;
    }
    

    _feetPosJumpStand << 0.185,  0.185,  -0.21,  -0.21,
                        -0.13,   0.13,   -0.13,   0.13,
                        -0.15,  -0.15,   -0.15,  -0.15;

    _robVelLimitX<<-0.4,0.4;
    _robVelLimitY<<-0.4,0.4;
    _robVelLimitYaw<< - 1.5,1.5; // 20度

}
