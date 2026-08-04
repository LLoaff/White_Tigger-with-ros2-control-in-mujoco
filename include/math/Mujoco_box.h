#ifndef MUJOCO_BOX_H
#define MUJOCO_BOX_H

#include "FSM/ControlComponent.h"

class Mujoco_box
{
public:
    Mujoco_box(ControlComponent* comp);
    ~Mujoco_box();
    void BoxUpdate();
private:
    int box_body_id = -1;
    int box_mocap_id = -1;

    double estimate_x;
    double estimate_y;
    double estimate_z;

    double estimate_qw;
    double estimate_qx;
    double estimate_qy;
    double estimate_qz;

    int _pos_index = -1;
    int _quat_index= -1;
    ControlComponent* _comp;
};




#endif

