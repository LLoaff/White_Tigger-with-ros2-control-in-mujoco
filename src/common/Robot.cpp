#include "Robot.h"

QuadrupedRobot::QuadrupedRobot(){
    _feetPosNormalStand << 0.185,  0.185,  -0.22,  -0.22,
                          -0.127,   0.127,   -0.127,  0.127,
                          -0.21,  -0.21,   -0.21,  -0.21;

    _feetPosJumpStand << 0.185,  0.185,  -0.21,  -0.21,
                        -0.13,   0.13,   -0.13,   0.13,
                        -0.15,  -0.15,   -0.15,  -0.15;

    _robVelLimitX<<-0.4,0.4;
    _robVelLimitY<<-0.4,0.4;
    _robVelLimitYaw<< - 1.5,1.5; // 20度

}


