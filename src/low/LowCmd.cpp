#include "low/LowCmd.h"

// #define LOWSTATE_DEBUG
//  #define LOWCMD_DEBUG
#define REDUCTION 1.0

LowCmd::LowCmd():serial(std::make_shared<SerialPort>("/dev/ttyACM0", B921600)),_motor_cmd(serial){
    for(int i = 0; i < 12; ++i){
        _motor_cmd.addMotor(&this->_state._motor_data[i]);
        // _motor_cmd.switchControlMode(this->_state._motor_data[i],damiao::MIT_MODE);
        // _motor_cmd.set_zero_position(this->_state._motor_data[i]); // 设置当前角度为零点
        // _motor_cmd.control_mit(this->_state._motor_data[i], 0, 0, 0, 0, 0);

        this->_state._motor_state[i].id = i;
        _cmd[i].id = i;
        _cmd[i].dq = 0;
        _cmd[i].tau = 0;
        _cmd[i].kp = 0;
        _cmd[i].kd = 0;

        switch (i)
        {
            case 0:
                _cmd[i].q = _state.Angle_Initialization_Variable.fr_hip_joint;
                break;
            case 1:
                _cmd[i].q = _state.Angle_Initialization_Variable.fr_knee_joint;
                break;
            case 2:
                _cmd[i].q = _state.Angle_Initialization_Variable.fr_thigh_joint;
                break;
            case 3:
                _cmd[i].q = _state.Angle_Initialization_Variable.fl_hip_joint;
                break;
            case 4:
                _cmd[i].q = _state.Angle_Initialization_Variable.fl_knee_joint;
                break;
            case 5:
                _cmd[i].q = _state.Angle_Initialization_Variable.fl_thigh_joint;
                break;
            case 6:
                _cmd[i].q = _state.Angle_Initialization_Variable.br_hip_joint;
                break;
            case 7:
                _cmd[i].q = _state.Angle_Initialization_Variable.br_knee_joint;
                break;
            case 8:
                _cmd[i].q = _state.Angle_Initialization_Variable.br_thigh_joint;
                break;
            case 9:
                _cmd[i].q = _state.Angle_Initialization_Variable.bl_hip_joint;
                break;
            case 10:
                _cmd[i].q = _state.Angle_Initialization_Variable.bl_knee_joint;
                break;
            case 11:
                _cmd[i].q = _state.Angle_Initialization_Variable.bl_thigh_joint;
                break;
            default:
                break;
        }
    }
    for(int i = 0; i < 12; ++i){
        _motor_cmd.enable(this->_state._motor_data[i]); // 使能
    }
    std::cout<<"LowCmd Init Success!"<<std::endl;
}

void LowCmd::Update(){
    struct Motor_State tmp_cmd[12];
    for(int i = 0; i < 12; ++i){
        tmp_cmd[i].id = i;
        tmp_cmd[i].q = _cmd[i].q;
        tmp_cmd[i].dq = _cmd[i].dq;
        tmp_cmd[i].tau = _cmd[i].tau;
        tmp_cmd[i].kp = _cmd[i].kp;
        tmp_cmd[i].kd = _cmd[i].kd;
        switch (i){
        case 0:
            tmp_cmd[i].q = (_cmd[i].q - _state.Angle_Initialization_Variable.fr_hip_joint)*REDUCTION;
            tmp_cmd[i].dq = _cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = _cmd[i].tau;
            break;
        case 1:
            tmp_cmd[i].q = -(_cmd[i].q - _state.Angle_Initialization_Variable.fr_knee_joint)*REDUCTION;
            tmp_cmd[i].dq = -_cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = -_cmd[i].tau;
            break;
        case 2:
            tmp_cmd[i].q = -(_cmd[i].q - _state.Angle_Initialization_Variable.fr_thigh_joint)*REDUCTION;
            tmp_cmd[i].dq = -_cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau =-_cmd[i].tau;
            break;
        case 3:
            tmp_cmd[i].q = (_cmd[i].q - _state.Angle_Initialization_Variable.fl_hip_joint)*REDUCTION;
            tmp_cmd[i].dq = _cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = _cmd[i].tau;
            break;
        case 4:
            tmp_cmd[i].q = (_cmd[i].q - _state.Angle_Initialization_Variable.fl_knee_joint)*REDUCTION;
            tmp_cmd[i].dq = _cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = _cmd[i].tau;
            break;
        case 5:
            tmp_cmd[i].q = (_cmd[i].q - _state.Angle_Initialization_Variable.fl_thigh_joint)*REDUCTION;
            tmp_cmd[i].dq = _cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = _cmd[i].tau;
            break;
        case 6:
            tmp_cmd[i].q = -(_cmd[i].q - _state.Angle_Initialization_Variable.br_hip_joint)*REDUCTION;
            tmp_cmd[i].dq = -_cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau =-_cmd[i].tau;
            break;
        case 7:
            tmp_cmd[i].q = -(_cmd[i].q - _state.Angle_Initialization_Variable.br_knee_joint)*REDUCTION;
            tmp_cmd[i].dq = -_cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = -_cmd[i].tau;
            break;
        case 8:
            tmp_cmd[i].q = -(_cmd[i].q - _state.Angle_Initialization_Variable.br_thigh_joint)*REDUCTION;
            tmp_cmd[i].dq = -_cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = -_cmd[i].tau;
            break;
        case 9:
            tmp_cmd[i].q = -(_cmd[i].q - _state.Angle_Initialization_Variable.bl_hip_joint)*REDUCTION;
            tmp_cmd[i].dq = -_cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = -_cmd[i].tau;
            break;
        case 10:
            tmp_cmd[i].q = (_cmd[i].q - _state.Angle_Initialization_Variable.bl_knee_joint)*REDUCTION;
            tmp_cmd[i].dq = _cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = _cmd[i].tau;
            break;
        case 11:
            tmp_cmd[i].q = (_cmd[i].q - _state.Angle_Initialization_Variable.bl_thigh_joint)*REDUCTION;
            tmp_cmd[i].dq = _cmd[i].dq*REDUCTION;
            tmp_cmd[i].tau = _cmd[i].tau;
            break;
        default:
            break;
        }
        // std::cout<<"id: "<< i << "cmd_q: "<<tmp_cmd[i].q<<std::endl;
        _motor_cmd.control_mit(this->_state._motor_data[i], tmp_cmd[i].kp, tmp_cmd[i].kd, tmp_cmd[i].q, tmp_cmd[i].dq, tmp_cmd[i].tau);
        switch (i){
        case 0:
            this->_state._motor_state[i].q = _state.Angle_Initialization_Variable.fr_hip_joint+ this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq = this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau = this->_state._motor_data[i].Get_tau();
            break;
        case 1:
            this->_state._motor_state[i].q = _state.Angle_Initialization_Variable.fr_knee_joint - this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq =- this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau =- this->_state._motor_data[i].Get_tau();
            break;
        case 2:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.fr_thigh_joint - this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq = -this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau = -this->_state._motor_data[i].Get_tau();
            break;
        case 3:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.fl_hip_joint + this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq = this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau = this->_state._motor_data[i].Get_tau();
            break;
        case 4:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.fl_knee_joint + this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq = this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau = this->_state._motor_data[i].Get_tau();
            break;
        case 5:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.fl_thigh_joint + this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq = this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau = this->_state._motor_data[i].Get_tau();
            break;
        case 6:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.br_hip_joint - this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq =- this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau =- this->_state._motor_data[i].Get_tau();
            break;
        case 7:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.br_knee_joint - this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq =- this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau =- this->_state._motor_data[i].Get_tau();
            break;
        case 8:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.br_thigh_joint - this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq = -this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau = -this->_state._motor_data[i].Get_tau();
            break;
        case 9:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.bl_hip_joint - this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq =- this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau =- this->_state._motor_data[i].Get_tau();
            break;
        case 10:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.bl_knee_joint + this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq = this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau = this->_state._motor_data[i].Get_tau();
            break;
        case 11:
            this->_state._motor_state[i].q =_state.Angle_Initialization_Variable.bl_thigh_joint + this->_state._motor_data[i].Get_Position();
            this->_state._motor_state[i].dq = this->_state._motor_data[i].Get_Velocity();
            this->_state._motor_state[i].tau = this->_state._motor_data[i].Get_tau();
            break;
        default:
            break;
        }
    }
}

void LowCmd::SetQ(Eigen::Matrix<float,12,1> q){
    for(int i(0); i<12; ++i){
        _cmd[i].q = q(i);
    }
}

void LowCmd::SetQ(int id,float q)       // 单独控制某一电机角度
{
    _cmd[id].q = q;
}

void LowCmd::SetQ(int leg_id , Eigen::Matrix<float,3,1> q)
{  
    _cmd[3*leg_id +0 ].q = q(0);
    _cmd[3*leg_id +1 ].q = q(1);
    _cmd[3*leg_id +2 ].q = q(2); 
}

void LowCmd::SetDq(int leg_id,Eigen::Matrix<float,3,1> dq)
{
    _cmd[3*leg_id +0 ].dq = dq(0);
    _cmd[3*leg_id +1 ].dq = dq(1);
    _cmd[3*leg_id +2 ].dq = dq(2); 
}

void LowCmd::SetDq(Eigen::Matrix<float,12,1> dq)  
{  
    for(int i=0;i<12;i++)
    {
        _cmd[i].dq = dq(i);
    }
}

void LowCmd::SetP(int leg_id,Eigen::Matrix<float,3,1> p)
{
    _cmd[3*leg_id +0 ].kp = p(0);
    _cmd[3*leg_id +1 ].kp = p(1);
    _cmd[3*leg_id +2 ].kp = p(2); 
}

void LowCmd::SetP(Eigen::Matrix<float,12,1> p)  
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

void LowCmd::SetD(int leg_id,Eigen::Matrix<float,3,1> d)
{
    _cmd[3*leg_id +0 ].kd = d(0);
    _cmd[3*leg_id +1 ].kd = d(1);
    _cmd[3*leg_id +2 ].kd = d(2); 
}

void LowCmd::SetD(Eigen::Matrix<float,12,1> d)  
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

void LowCmd::SetTau(Eigen::Matrix<float,12,1> tau){
    for(int i(0); i<12; ++i){
        if(std::isnan(tau(i))){
            printf("[ERROR] The setTau function meets Nan\n");
            exit(-1);
        }
        _cmd[i].tau = tau(i);
    }
}

void LowCmd::SetTau(int leg_id,Eigen::Matrix<float,3,1> tau){
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

Eigen::Matrix<float,3,4> LowCmd::getQ(){
  Eigen::Matrix<float,3,4> qLegs;
  for(int i(0); i < 4; ++i){
      qLegs.col(i)(0) = _state._motor_state[3*i    ].q;
      qLegs.col(i)(1) = _state._motor_state[3*i + 1].q;
      qLegs.col(i)(2) = _state._motor_state[3*i + 2].q;
  }
  return qLegs;
}

Eigen::Matrix<float,12,1> LowCmd::getQ12(){
  Eigen::Matrix<float,12,1> qLegs;
  for(int i(0); i < 4; ++i){
      qLegs(3*i  ) = _state._motor_state[3*i  ].q;
      qLegs(3*i+1) = _state._motor_state[3*i+1].q;
      qLegs(3*i+2) = _state._motor_state[3*i+2].q;
  }
  return qLegs;
}

Eigen::Matrix<float,12,1> LowCmd::getW12(){
  Eigen::Matrix<float,12,1> w;
  for(int i(0); i < 4; ++i){
      w(3*i  ) = _state._motor_state[3*i  ].dq ;
      w(3*i+1) = _state._motor_state[3*i+1].dq ;
      w(3*i+2) = _state._motor_state[3*i+2].dq;
  }
  return w;
}

void LowCmd::setStableGain(int legID){
    // _cmd[legID*3+0].kp = 10.5;
    // _cmd[legID*3+0].kd = 2.5;

    // _cmd[legID*3+1].kp = 11.5;
    // _cmd[legID*3+1].kd = 2.5;

    // _cmd[legID*3+2].kp = 11.9;
    // _cmd[legID*3+2].kd = 2.5;
    _cmd[legID*3+0].kp = 9.0;
    _cmd[legID*3+0].kd = 1.5;

    _cmd[legID*3+1].kp = 11.5;
    _cmd[legID*3+1].kd = 1.5;

    _cmd[legID*3+2].kp = 11.8;
    _cmd[legID*3+2].kd = 1.5;
}
void LowCmd::setStableGain(){
    for(int i(0); i<4; ++i){
        setStableGain(i);
    }
}
void LowCmd::setSwingGain(int legID){
    // _cmd[legID*3+0].kp = 12.5;
    // _cmd[legID*3+0].kd = 2.0;

    // _cmd[legID*3+1].kp = 13.8;
    // _cmd[legID*3+1].kd = 2.0;

    // _cmd[legID*3+2].kp = 13.8;
    // _cmd[legID*3+2].kd = 2.0;
    _cmd[legID*3+0].kp = 8.5;
    _cmd[legID*3+0].kd = 0.4;

    _cmd[legID*3+1].kp = 9.8;
    _cmd[legID*3+1].kd = 0.6;

    _cmd[legID*3+2].kp = 9.8;
    _cmd[legID*3+2].kd = 0.8;
}

Eigen::Matrix<float,12,1> LowCmd::getInitialQ12(){
    Eigen::Matrix<float,12,1> initialQ;
    initialQ(0) = _state.Angle_Initialization_Variable.fr_hip_joint;
    initialQ(1) = _state.Angle_Initialization_Variable.fr_knee_joint;
    initialQ(2) = _state.Angle_Initialization_Variable.fr_thigh_joint;
    initialQ(3) = _state.Angle_Initialization_Variable.fl_hip_joint;
    initialQ(4) = _state.Angle_Initialization_Variable.fl_knee_joint;
    initialQ(5) = _state.Angle_Initialization_Variable.fl_thigh_joint;
    initialQ(6) = _state.Angle_Initialization_Variable.br_hip_joint;
    initialQ(7) = _state.Angle_Initialization_Variable.br_knee_joint;
    initialQ(8) = _state.Angle_Initialization_Variable.br_thigh_joint;
    initialQ(9) = _state.Angle_Initialization_Variable.bl_hip_joint;
    initialQ(10) = _state.Angle_Initialization_Variable.bl_knee_joint;
    initialQ(11) = _state.Angle_Initialization_Variable.bl_thigh_joint;
    return initialQ;
}
LowCmd::~LowCmd() {
    for (size_t i = 0; i < 12; i++){
        _motor_cmd.disable(this->_state._motor_data[i]); // 失能
    }
    
}    


