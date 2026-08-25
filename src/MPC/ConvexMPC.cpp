#include "MPC/ConvexMPC.h"

ConvexMPC::ConvexMPC(ControlComponent* comp):_comp(comp),_dt(comp->dt),iterationsBetweenMPC(50)
,horizonLength(10){

    Kp << 700, 0, 0,
        0, 700, 0,
        0, 0, 150;
    Kp_stance = 0*Kp;

    Kd << 7, 0, 0,
        0, 7, 0,
        0, 0, 7;
    Kd_stance = Kd;

}

void ConvexMPC::MPCrun(){

    Vec4 contactStates = _comp->waveGen->getContactState();
    Vec4 swingStates = _comp->waveGen->getSwingState();


}

ConvexMPC::~ConvexMPC(){



}

