#ifndef ROBOT_H
#define ROBOT_H

#include "math/mathTypes.h"

class QuadrupedRobot{
public:
    QuadrupedRobot();

    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}
    Vec34 getFeetPosJump(){return _feetPosJumpStand;}
    Vec34 getHipPos(){return _HipPos;}
    Vec4 getOffset(){return _Offset;}
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
protected:
    Vec34 _feetPosNormalStand;
    Vec34 _feetPosJumpStand;
    Vec34 _HipPos;
    Vec4 _Offset;
    Vec2 _robVelLimitX;
    Vec2 _robVelLimitY;
    Vec2 _robVelLimitYaw;
};
#endif  