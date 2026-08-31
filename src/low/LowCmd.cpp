#include "low/LowCmd.h"

// #define LOWSTATE_DEBUG
//  #define LOWCMD_DEBUG


LowCmd::LowCmd(mjModel *model, mjData *data):_mjmodel(model),_mjdata(data){
    idInit();
    _state = new LowState(model,data);
    for(int i = 0; i < 12; ++i){
        this->_state->_motor_state[i].id = i;
        _cmd[i].id = i;
        _cmd[i].dq = 0;
        _cmd[i].tau = 0;
        _cmd[i].kp = 0;
        _cmd[i].kd = 0;

        switch (i)
        {
            case 0:
                _cmd[i].q = _state->Angle_Initialization_Variable.fr_hip_joint;
                break;
            case 1:
                _cmd[i].q = _state->Angle_Initialization_Variable.fr_thigh_joint;
                break;
            case 2:
                _cmd[i].q = _state->Angle_Initialization_Variable.fr_calf_joint;
                break;
            case 3:
                _cmd[i].q = _state->Angle_Initialization_Variable.fl_hip_joint;
                break;
            case 4:
                _cmd[i].q = _state->Angle_Initialization_Variable.fl_thigh_joint;
                break;
            case 5:
                _cmd[i].q = _state->Angle_Initialization_Variable.fl_calf_joint;
                break;
            case 6:
                _cmd[i].q = _state->Angle_Initialization_Variable.br_hip_joint;
                break;
            case 7:
                _cmd[i].q = _state->Angle_Initialization_Variable.br_thigh_joint;
                break;
            case 8:
                _cmd[i].q = _state->Angle_Initialization_Variable.br_calf_joint;
                break;
            case 9:
                _cmd[i].q = _state->Angle_Initialization_Variable.bl_hip_joint;
                break;
            case 10:
                _cmd[i].q = _state->Angle_Initialization_Variable.bl_thigh_joint;
                break;
            case 11:
                _cmd[i].q = _state->Angle_Initialization_Variable.bl_calf_joint;
                break;
            default:
                break;
        }
    }
    std::cout<<"LowCmd Init Success!"<<std::endl;
}

void LowCmd::Update(){
    struct Motor_State tmp_cmd[12];
    for(int i = 0; i < 12; ++i){
        // tmp_cmd[i].id = i;
        // tmp_cmd[i].q = _cmd[i].q;
        // tmp_cmd[i].dq = _cmd[i].dq;
        // tmp_cmd[i].tau = _cmd[i].tau;
        // tmp_cmd[i].kp = _cmd[i].kp;
        // tmp_cmd[i].kd = _cmd[i].kd;

        _state->_motor_state[i].q = _mjdata->sensordata[_mjmodel->sensor_adr[_jointid[3*i + 0]]];
        _state->_motor_state[i].dq = _mjdata->sensordata[_mjmodel->sensor_adr[_jointid[3*i + 1]]];
        _state->_motor_state[i].tau = _mjdata->sensordata[_mjmodel->sensor_adr[_jointid[3*i + 2]]];
        // std::cout<<"id :"<<i<<" q: "<< _state->_motor_state[i].q<<std::endl;
        // std::cout<<"id :"<<i<<" v: "<< _state->_motor_state[i].dq<<std::endl;
        // std::cout<<"id :"<<i<<" t: "<< _state->_motor_state[i].tau<<std::endl;
        this->PDController(i,_cmd[i].q,_cmd[i].dq,_cmd[i].kp,_cmd[i].kd,_cmd[i].tau);
    }
}

void LowCmd::PDController(int id,float q,float vel,float kp,float kd,float tau){
    float q_cur = this->_state->_motor_state[id].q;
    float dq_cur = this->_state->_motor_state[id].dq;
    float out_tau = tau + kp*(q-q_cur)+kd*(vel-dq_cur);
    // std::cout<<"id: "<<id<<" tau: "<< out_tau<<std::endl;
    if (std::isnan(out_tau) || std::isinf(out_tau) || fabs(out_tau) > 1000) {
        printf("[ERROR] 电机%d 控制量非法! out_tau=%.6f\n", id, out_tau);
        printf("  输入: q_des=%.6f, q_cur=%.6f, dq_des=%.6f, dq_cur=%.6f\n", 
               q, q_cur, vel, dq_cur);
        printf("  参数: kp=%.6f, kd=%.6f, tau_ff=%.6f\n", kp, kd, tau);
    }
    this->_mjdata->ctrl[id] = out_tau;
}

void LowCmd::SetQ(Eigen::Matrix<double,12,1> q){
    for(int i(0); i<12; ++i){
        _cmd[i].q = q(i);
    }
}

void LowCmd::SetQ(int id,float q)       // 单独控制某一电机角度
{
    _cmd[id].q = q;
}

void LowCmd::SetQ(int leg_id , Eigen::Matrix<double,3,1> q)
{  
    _cmd[3*leg_id +0 ].q = q(0);
    _cmd[3*leg_id +1 ].q = q(1);
    _cmd[3*leg_id +2 ].q = q(2); 
}

void LowCmd::SetDq(int leg_id,Eigen::Matrix<double,3,1> dq)
{
    _cmd[3*leg_id +0 ].dq = dq(0);
    _cmd[3*leg_id +1 ].dq = dq(1);
    _cmd[3*leg_id +2 ].dq = dq(2); 
}

void LowCmd::SetDq(Eigen::Matrix<double,12,1> dq)  
{  
    for(int i=0;i<12;i++)
    {
        _cmd[i].dq = dq(i);
    }
}

void LowCmd::SetP(int leg_id,Eigen::Matrix<double,3,1> p)
{
    _cmd[3*leg_id +0 ].kp = p(0);
    _cmd[3*leg_id +1 ].kp = p(1);
    _cmd[3*leg_id +2 ].kp = p(2); 
}

void LowCmd::SetP(Eigen::Matrix<double,12,1> p)  
{  
    for(int i=0;i<12;i++)
    {
        _cmd[i].kp = p(i);
    }
}

void LowCmd::SetZeroP(){
    for(int i(0); i<4; ++i){
        _cmd[3*i+0].kp = 0;
        _cmd[3*i+1].kp = 0;
        _cmd[3*i+2].kp = 0;
    }
}

void LowCmd::SetD(int leg_id,Eigen::Matrix<double,3,1> d)
{
    _cmd[3*leg_id +0 ].kd = d(0);
    _cmd[3*leg_id +1 ].kd = d(1);
    _cmd[3*leg_id +2 ].kd = d(2); 
}

void LowCmd::SetD(Eigen::Matrix<double,12,1> d)  
{  
    for(int i=0;i<12;i++)
    {
        _cmd[i].kd = d(i);
    }
}

void LowCmd::SetZeroD(){
    for(int i(0); i<4; ++i){
        _cmd[3*i+0].kd = 0;
        _cmd[3*i+1].kd = 0;
        _cmd[3*i+2].kd = 0;
    }
}

void LowCmd::SetTau(Eigen::Matrix<double,12,1> tau){
    for(int i(0); i<12; ++i){
        if(std::isnan(tau(i))){
            printf("[ERROR] The setTau function meets Nan\n");
            exit(-1);
        }
        _cmd[i].tau = tau(i);
    }
}

void LowCmd::SetTau(int leg_id,Eigen::Matrix<double,3,1> tau){
    _cmd[leg_id*3+0].tau = tau(0);
    _cmd[leg_id*3+1].tau = tau(1);
    _cmd[leg_id*3+2].tau = tau(2);
    
}

void LowCmd::SetZeroTau(int legID){
    _cmd[legID*3+0].tau = 0;
    _cmd[legID*3+1].tau = 0;
    _cmd[legID*3+2].tau = 0;
}

void LowCmd::SetZeroTau(){
    for(uint8_t i=0;i<4;i++)
    {
        _cmd[i*3+0].tau = 0;
        _cmd[i*3+1].tau = 0;
        _cmd[i*3+2].tau = 0;
    }
    
}

void LowCmd::SetZeroDq(int legID){
    _cmd[legID*3+0].dq = 0;
    _cmd[legID*3+1].dq = 0;
    _cmd[legID*3+2].dq = 0;
}

void LowCmd::SetZeroDq(){
    for(int i(0); i<4; ++i){
        SetZeroDq(i);
    }
}

void LowCmd::SetFree()
{
    SetZeroDq();
    SetZeroTau();
}

Eigen::Matrix<double,3,4> LowCmd::getQ(){
  Eigen::Matrix<double,3,4> qLegs;
  for(int i(0); i < 4; ++i){
      qLegs.col(i)(0) = _state->_motor_state[3*i    ].q;
      qLegs.col(i)(1) = _state->_motor_state[3*i + 1].q;
      qLegs.col(i)(2) = _state->_motor_state[3*i + 2].q;
  }
  return qLegs;
}

Eigen::Matrix<double,12,1> LowCmd::getQ12(){
  Eigen::Matrix<double,12,1> qLegs;
  for(int i(0); i < 4; ++i){
      qLegs(3*i  ) = _state->_motor_state[3*i  ].q;
      qLegs(3*i+1) = _state->_motor_state[3*i+1].q;
      qLegs(3*i+2) = _state->_motor_state[3*i+2].q;
  }
  return qLegs;
}

Eigen::Matrix<double,12,1> LowCmd::getW12(){
  Eigen::Matrix<double,12,1> w;
  for(int i(0); i < 4; ++i){
      w(3*i  ) = _state->_motor_state[3*i  ].dq ;
      w(3*i+1) = _state->_motor_state[3*i+1].dq ;
      w(3*i+2) = _state->_motor_state[3*i+2].dq;
  }
  return w;
}

void LowCmd::setStableGain(int legID){
    if(use_go1_model ==1 ){
        _cmd[legID*3+0].kp = 7;
        _cmd[legID*3+0].kd = 2.0;

        _cmd[legID*3+1].kp = 7;
        _cmd[legID*3+1].kd = 2.0;

        _cmd[legID*3+2].kp = 7;
        _cmd[legID*3+2].kd = 2.0;
    }else{
        _cmd[legID*3+0].kp = 4.5;
        _cmd[legID*3+0].kd = 1.1;

        _cmd[legID*3+1].kp = 4.5;
        _cmd[legID*3+1].kd = 1.1;

        _cmd[legID*3+2].kp = 4.5;
        _cmd[legID*3+2].kd = 1.1;
    }
}
void LowCmd::setStableGain(){
    for(int i(0); i<4; ++i){
        setStableGain(i);
    }
}
void LowCmd::setSwingGain(int legID){
    if(use_go1_model ==1 ){
        _cmd[legID*3+0].kp = 5.5;
        _cmd[legID*3+0].kd = 1;

        _cmd[legID*3+1].kp = 5.5;
        _cmd[legID*3+1].kd = 1;

        _cmd[legID*3+2].kp = 5.5;
        _cmd[legID*3+2].kd = 1;
    }else{
        _cmd[legID*3+0].kp = 3.5;
        _cmd[legID*3+0].kd = 0.7;

        _cmd[legID*3+1].kp = 3.5;
        _cmd[legID*3+1].kd = 0.7;

        _cmd[legID*3+2].kp = 3.5;
        _cmd[legID*3+2].kd = 0.7;
    }
}
void LowCmd::setStableGain_JUMP(int legID){
    _cmd[legID*3+0].kp = 4.5;
    _cmd[legID*3+0].kd = 1.5;

    _cmd[legID*3+1].kp = 4.5;
    _cmd[legID*3+1].kd = 1.5;

    _cmd[legID*3+2].kp = 4.9;
    _cmd[legID*3+2].kd = 1.5;
}
void LowCmd::setSwingGain_JUMP(int legID){
    _cmd[legID*3+0].kp = 4.5;
    _cmd[legID*3+0].kd = 1.5;

    _cmd[legID*3+1].kp = 4.8;
    _cmd[legID*3+1].kd = 1.5;

    _cmd[legID*3+2].kp = 4.8;
    _cmd[legID*3+2].kd = 1.5;
}
void LowCmd::setZeroGain(int legID){
    _cmd[legID*3+0].kp = 0;
    _cmd[legID*3+0].kd = 0;

    _cmd[legID*3+1].kp = 0;
    _cmd[legID*3+1].kd = 0;

    _cmd[legID*3+2].kp = 0;
    _cmd[legID*3+2].kd = 0;
}
Eigen::Matrix<double,12,1> LowCmd::getInitialQ12(){
    Eigen::Matrix<double,12,1> initialQ;
    initialQ(0) = _state->Angle_Initialization_Variable.fr_hip_joint;
    initialQ(1) = _state->Angle_Initialization_Variable.fr_thigh_joint;
    initialQ(2) = _state->Angle_Initialization_Variable.fr_calf_joint;
    initialQ(3) = _state->Angle_Initialization_Variable.fl_hip_joint;
    initialQ(4) = _state->Angle_Initialization_Variable.fl_thigh_joint;
    initialQ(5) = _state->Angle_Initialization_Variable.fl_calf_joint;
    initialQ(6) = _state->Angle_Initialization_Variable.br_hip_joint;
    initialQ(7) = _state->Angle_Initialization_Variable.br_thigh_joint;
    initialQ(8) = _state->Angle_Initialization_Variable.br_calf_joint;
    initialQ(9) = _state->Angle_Initialization_Variable.bl_hip_joint;
    initialQ(10) = _state->Angle_Initialization_Variable.bl_thigh_joint;
    initialQ(11) = _state->Angle_Initialization_Variable.bl_calf_joint;
    return initialQ;
}
LowCmd::~LowCmd() {
}    

void LowCmd::idInit(){
    if(use_go1_model == 1){
        _jointid[0] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_hip_joint_p");
        _jointid[1] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_hip_joint_v");
        _jointid[2] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_hip_joint_f");

        _jointid[3] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_thigh_joint_p");
        _jointid[4] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_thigh_joint_v");
        _jointid[5] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_thigh_joint_f");

        _jointid[6] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_calf_joint_p");
        _jointid[7] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_calf_joint_v");
        _jointid[8] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FR_calf_joint_f");

        _jointid[9] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_hip_joint_p");
        _jointid[10] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_hip_joint_v");
        _jointid[11] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_hip_joint_f");

        _jointid[12] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_thigh_joint_p");
        _jointid[13] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_thigh_joint_v");
        _jointid[14] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_thigh_joint_f");

        _jointid[15] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_calf_joint_p");
        _jointid[16] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_calf_joint_v");
        _jointid[17] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"FL_calf_joint_f");

        _jointid[18] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_hip_joint_p");
        _jointid[19] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_hip_joint_v");
        _jointid[20] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_hip_joint_f");

        _jointid[21] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_thigh_joint_p");
        _jointid[22] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_thigh_joint_v");
        _jointid[23] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_thigh_joint_f");

        _jointid[24] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_calf_joint_p");
        _jointid[25] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_calf_joint_v");
        _jointid[26] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RR_calf_joint_f");

        _jointid[27] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_hip_joint_p");
        _jointid[28] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_hip_joint_v");
        _jointid[29] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_hip_joint_f");

        _jointid[30] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_thigh_joint_p");
        _jointid[31] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_thigh_joint_v");
        _jointid[32] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_thigh_joint_f");

        _jointid[33] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_calf_joint_p");
        _jointid[34] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_calf_joint_v");
        _jointid[35] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"RL_calf_joint_f");
    }
    else{
        _jointid[0] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_hip_joint_p");
        _jointid[1] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_hip_joint_v");
        _jointid[2] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_hip_joint_f");

        _jointid[3] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_thigh_joint_p");
        _jointid[4] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_thigh_joint_v");
        _jointid[5] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_thigh_joint_f");

        _jointid[6] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_calf_joint_p");
        _jointid[7] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_calf_joint_v");
        _jointid[8] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fr_calf_joint_f");

        _jointid[9] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_hip_joint_p");
        _jointid[10] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_hip_joint_v");
        _jointid[11] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_hip_joint_f");

        _jointid[12] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_thigh_joint_p");
        _jointid[13] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_thigh_joint_v");
        _jointid[14] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_thigh_joint_f");

        _jointid[15] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_calf_joint_p");
        _jointid[16] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_calf_joint_v");
        _jointid[17] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"fl_calf_joint_f");

        _jointid[18] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_hip_joint_p");
        _jointid[19] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_hip_joint_v");
        _jointid[20] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_hip_joint_f");

        _jointid[21] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_thigh_joint_p");
        _jointid[22] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_thigh_joint_v");
        _jointid[23] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_thigh_joint_f");

        _jointid[24] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_calf_joint_p");
        _jointid[25] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_calf_joint_v");
        _jointid[26] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"br_calf_joint_f");

        _jointid[27] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_hip_joint_p");
        _jointid[28] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_hip_joint_v");
        _jointid[29] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_hip_joint_f");

        _jointid[30] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_thigh_joint_p");
        _jointid[31] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_thigh_joint_v");
        _jointid[32] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_thigh_joint_f");

        _jointid[33] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_calf_joint_p");
        _jointid[34] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_calf_joint_v");
        _jointid[35] = mj_name2id(_mjmodel,mjOBJ_SENSOR,"bl_calf_joint_f");
    }
}

