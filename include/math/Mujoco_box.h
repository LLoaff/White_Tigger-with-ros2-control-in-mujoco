#ifndef MUJOCO_BOX_H
#define MUJOCO_BOX_H

#ifdef USE_SIM
#include "FSM/ControlComponent.h"
#include "math/mathTypes.h"

class Mujoco_box
{
public:
    Mujoco_box(ControlComponent* comp);
    ~Mujoco_box();
    void BoxUpdate(Vec3  pcom,Vec4 quat);
private:
    int box_body_id = -1;
    int box_mocap_id = -1;

    // float estimate_x;
    // float estimate_y;
    // float estimate_z;

    // float estimate_qw;
    // float estimate_qx;
    // float estimate_qy;
    // float estimate_qz;

    int _pos_index = -1;
    int _quat_index= -1;
    ControlComponent* _comp;
};

#endif


#endif

