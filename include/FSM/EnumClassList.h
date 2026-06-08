#ifndef ENUMCLASSLIST_H
#define ENUMCLASSLIST_H


enum class UserValue
{
    PASSIVE,
    FREE,
    STAND,
    FREE_STAND,
    BALANCE,
    TROTTING,
    SIT_DOWN,
    JUMP,
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    INVALID,
    PASSIVE,
    FREE,
    STAND,
    FREE_STAND,
    BALANCE,
    TROTTING,
    SIT_DOWN,
    JUMP,
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};
#endif