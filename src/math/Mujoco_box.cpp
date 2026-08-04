#include "math/Mujoco_box.h"

Mujoco_box::Mujoco_box(ControlComponent* comp):_comp(comp){
    box_body_id = mj_name2id(_comp->_mjmodel , mjOBJ_BODY, "estimator_box");
    box_mocap_id = _comp->_mjmodel->body_mocapid[box_body_id];
    if (box_body_id == -1) {
        std::cerr << "没有找到 estimator_box body" << std::endl;
        return;
    }
    if (box_mocap_id == -1) {
        std::cerr << "estimator_box 不是 mocap body" << std::endl;
        return;
    }

    int _pos_index = 3 * box_mocap_id;
    int _quat_index = 4 * box_mocap_id;
}

void Mujoco_box::BoxUpdate(){
    _comp->_mjdata->mocap_pos[_pos_index + 0] = _comp->_mjBox_pos[0];
    _comp->_mjdata->mocap_pos[_pos_index + 1] = _comp->_mjBox_pos[1];
    _comp->_mjdata->mocap_pos[_pos_index + 2] = _comp->_mjBox_pos[2];

    _comp->_mjdata->mocap_quat[_pos_index + 0] = _comp->_mjBox_pos[3];
    _comp->_mjdata->mocap_quat[_pos_index + 1] = _comp->_mjBox_pos[4];
    _comp->_mjdata->mocap_quat[_pos_index + 2] = _comp->_mjBox_pos[5];
    _comp->_mjdata->mocap_quat[_pos_index + 3] = _comp->_mjBox_pos[6];

}

Mujoco_box::~Mujoco_box(){


}