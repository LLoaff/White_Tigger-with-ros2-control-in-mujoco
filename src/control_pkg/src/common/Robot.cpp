#include "Robot.h"
#include "mathtool.h"

QuadrupedRobot::QuadrupedRobot(){
    _feetPosNormalStand << _length_-0.02, _length_-0.02, -_length_-0.04, -_length_-0.04,
                          -_weigh_ - _labad_ - 0.02, _weigh_ + _labad_+0.02, -_weigh_ - _labad_, _weigh_ + _labad_,
                          -0.21, -0.21, -0.21, -0.21;

    _feetPosJumpStand << 0.185,  0.185,  -0.21,  -0.21,
                        -0.13,   0.13,   -0.13,   0.13,
                        -0.15,  -0.15,   -0.15,  -0.15;

    _robVelLimitX<<-0.4,0.4;
    _robVelLimitY<<-0.4,0.4;
    _robVelLimitYaw<< - 1.5,1.5; // 20度

}
