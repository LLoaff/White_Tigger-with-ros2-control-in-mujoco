#include "IOPort/IORos.h"

void RosShutDown(int sig){
	rclcpp::shutdown();
}

IORos::IORos():Node("IORos"){
    _joint_puber_=this->create_publisher<motor_control_msgs::msg::DmInterface>("/dm_command",1);

    _joint_cmd = std::make_shared<motor_control_msgs::msg::DmInterface>();
    _joint_cmd->id.resize(12);
    _joint_cmd->kp.resize(12);
    _joint_cmd->kd.resize(12);
    _joint_cmd->q.resize(12);
    _joint_cmd->dq.resize(12);
    _joint_cmd->tau.resize(12);

    _state._imu.quaternion[0]=1;
    _state._imu.quaternion[1]=0;
    _state._imu.quaternion[2]=0;
    _state._imu.quaternion[3]=0;

    _joint_cmd-> q[0] =_state.Angle_Initialization_Variable.fr_hip_joint;
    _joint_cmd-> q[1] =_state.Angle_Initialization_Variable.fr_thigh_joint;
    _joint_cmd-> q[2] =_state.Angle_Initialization_Variable.fr_calf_joint;

    _joint_cmd-> q[3] =_state.Angle_Initialization_Variable.fl_hip_joint;
    _joint_cmd-> q[4] =_state.Angle_Initialization_Variable.fl_thigh_joint;
    _joint_cmd-> q[5] =_state.Angle_Initialization_Variable.fl_calf_joint;

    _joint_cmd-> q[6] =_state.Angle_Initialization_Variable.br_hip_joint;
    _joint_cmd-> q[7] =_state.Angle_Initialization_Variable.br_thigh_joint;
    _joint_cmd-> q[8] =_state.Angle_Initialization_Variable.br_calf_joint;

    _joint_cmd-> q[9] =_state.Angle_Initialization_Variable.bl_hip_joint;
    _joint_cmd-> q[10] =_state.Angle_Initialization_Variable.bl_thigh_joint;
    _joint_cmd-> q[11] =_state.Angle_Initialization_Variable.bl_calf_joint;

    for(int i(0);i<12;++i){
        _joint_cmd->id[i]=i;
        _joint_cmd->dq[i] = 0.0f;
        _joint_cmd->tau[i] = 0.0f;
        _joint_cmd->kp[i] = 0.0f;
        _joint_cmd->kd[i] = 0.0f;
        _init_q(i) = _joint_cmd-> q[i];
        _state._motor_state[i].id = i;
        _state._motor_state[i].q = _joint_cmd->q[i];
        _state._motor_state[i].dq = 0.0f;
        _state._motor_state[i].tau = 0.0f;
        _state._motor_state[i].kp = 0.0f;
        _state._motor_state[i].kd = 0.0f;
    }
    for(int i(0); i<6; ++i){
        _cmd_vel[i] = 0.0f;
    }

    signal(SIGINT, RosShutDown);
    init();
}

void IORos::init(){
    _cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",1,
        std::bind(&IORos::cmdVelCallBack,this,std::placeholders::_1));
    _joint_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",1,
        std::bind(&IORos::stateCallBack,this,std::placeholders::_1));
    _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu_sensor_broadcaster/imu",1,
        std::bind(&IORos::imuCallBack,this,std::placeholders::_1));
}

void IORos::upDate(){
    // for(int i(0);i<12;++i){
    //     _joint_cmd-> q[i] = _motor_cmd[i].q;
    //     _joint_cmd-> dq[i] = _motor_cmd[i].dq;
    // }
    _joint_cmd->header.stamp = this->now();
    _joint_puber_->publish(*_joint_cmd);
}

void IORos::stateCallBack(const sensor_msgs::msg::JointState & rec){
    std::unordered_map<std::string, size_t> name_to_idx_map;
    for (size_t i = 0; i < rec.name.size(); ++i) {
        name_to_idx_map[rec.name[i]] = i;
    }
    for (size_t idx = 0; idx < _joint_order.size(); ++idx) {
        const std::string &joint_name = _joint_order[idx];
        auto it = name_to_idx_map.find(joint_name);
        if (it!= name_to_idx_map.end()) {
            size_t msg_idx = it->second; // 获取在消息中的真实索引

            if (msg_idx < rec.position.size() && msg_idx < rec.velocity.size()) {
                _state._motor_state[idx].q = rec.position[msg_idx];  // 角度
                _state._motor_state[idx].dq = rec.velocity[msg_idx]; // 角速度
            }
            } 
        }
    
}

void IORos::imuCallBack(const sensor_msgs::msg::Imu& rec){
    _state._imu.accelerometer[0] = rec.linear_acceleration.x;
    _state._imu.accelerometer[1] = rec.linear_acceleration.y;
    _state._imu.accelerometer[2] = rec.linear_acceleration.z;

    _state._imu.gyroscope[0] = rec.angular_velocity.x;
    _state._imu.gyroscope[1] = rec.angular_velocity.y;
    _state._imu.gyroscope[2] = rec.angular_velocity.z;

    _state._imu.quaternion[0] = rec.orientation.w;
    _state._imu.quaternion[1] = rec.orientation.x;
    _state._imu.quaternion[2] = rec.orientation.y;
    _state._imu.quaternion[3] = rec.orientation.z;
}

Eigen::Matrix<float,3,4> IORos::getQ(){
    Eigen::Matrix<float,3,4> qLegs;
    for(int i(0); i < 4; ++i){
        qLegs.col(i)(0) = _state._motor_state[3*i + 0].q;
        qLegs.col(i)(1) = _state._motor_state[3*i + 1].q;
        qLegs.col(i)(2) = _state._motor_state[3*i + 2].q;
    }
  return qLegs;
}
void IORos::cmdVelCallBack(const geometry_msgs::msg::Twist& rec){
    _cmd_vel[0] = rec.linear.x;
    _cmd_vel[1] = rec.linear.y;
    _cmd_vel[2] = rec.linear.z;
    _cmd_vel[3] = rec.angular.x;
    _cmd_vel[4] = rec.angular.y;
    _cmd_vel[5] = rec.angular.z;
}
Eigen::Matrix<float,12,1> IORos::getQ12(){
    Eigen::Matrix<float,12,1> qLegs;
    for(int i(0); i < 4; ++i){
        qLegs(3*i  ) = _state._motor_state[3*i + 0].q;
        qLegs(3*i+1) = _state._motor_state[3*i + 1].q;
        qLegs(3*i+2) = _state._motor_state[3*i + 2].q;
    }
  return qLegs;
}

Eigen::Matrix<float,12,1> IORos::getW12(){
    Eigen::Matrix<float,12,1> w;
    for(int i(0); i < 4; ++i){
        w(3*i  ) = _state._motor_state[3*i + 0].dq;
        w(3*i+1) = _state._motor_state[3*i + 1].dq;
        w(3*i+2) = _state._motor_state[3*i + 2].dq;
    }
  return w;
}

void IORos::SetQ(Eigen::Matrix<float,12,1> q)
{
    for(int i(0); i<12; ++i){
        _joint_cmd-> q[i] = q(i);
    }
}

void IORos::SetQ(int id,float q)       // 单独控制某一电机角度
{
    _joint_cmd-> q[id] = q;
}

void IORos::SetQ(int leg_id , Eigen::Matrix<float,3,1> q)
{
    _joint_cmd-> q[3*leg_id +0 ] = q(0);
    _joint_cmd-> q[3*leg_id +1 ] = q(1);
    _joint_cmd-> q[3*leg_id +2 ] = q(2); 
}

void IORos::SetDq(int leg_id,Eigen::Matrix<float,3,1> dq)
{
    _joint_cmd-> dq[3*leg_id +0 ] = dq(0);
    _joint_cmd-> dq[3*leg_id +1 ] = dq(1);
    _joint_cmd-> dq[3*leg_id +2 ] = dq(2); 
}

void IORos::SetDq(Eigen::Matrix<float,12,1> dq)  
{  
    for(int i=0;i<12;i++)
    {
        _joint_cmd-> dq[i] = dq(i);
    }
}

void IORos::SetP(int leg_id,Eigen::Matrix<float,3,1> p)
{
    _joint_cmd->kp[3*leg_id +0 ] = p(0);
    _joint_cmd->kp[3*leg_id +1 ] = p(1);
    _joint_cmd->kp[3*leg_id +2 ] = p(2); 
}

void IORos::SetP(Eigen::Matrix<float,12,1> p)  
{  
    for(int i=0;i<12;i++)
    {
        _joint_cmd->kp[i] = p(i);
    }
}

void IORos::SetZeroP(){
    for(int i(0); i<4; ++i){
        _joint_cmd->kp[3*i+0] = 0;
        _joint_cmd->kp[3*i+1] = 0;
        _joint_cmd->kp[3*i+2] = 0;
    }
}

void IORos::SetD(int leg_id,Eigen::Matrix<float,3,1> d)
{
    _joint_cmd->kd[3*leg_id +0 ] = d(0);
    _joint_cmd->kd[3*leg_id +1 ] = d(1);
    _joint_cmd->kd[3*leg_id +2 ] = d(2); 
}

void IORos::SetD(Eigen::Matrix<float,12,1> d)  
{  
    for(int i=0;i<12;i++)
    {
        _joint_cmd->kd[i] = d(i);
    }
}

void IORos::SetZeroD(){
    for(int i(0); i<4; ++i){
        _joint_cmd->kd[3*i+0] = 0;
        _joint_cmd->kd[3*i+1] = 0;
        _joint_cmd->kd[3*i+2] = 0;
    }
}
                                                                                                                    // -2.3, 2.3
void IORos::SetTau(Eigen::Matrix<float,12,1> tau,Eigen::Matrix<double,2,1> torqueLimit){
    for(int i(0); i<12; ++i){
        if(std::isnan(tau(i))){
            printf("[ERROR] The setTau function meets Nan\n");
            exit(1);
        }
        _joint_cmd-> tau[i] = saturation(tau(i),torqueLimit);
    }
}

void IORos::SetTau(int leg_id,Eigen::Matrix<float,3,1> tau){
    _joint_cmd-> tau[leg_id*3+0] = tau(0);
    _joint_cmd-> tau[leg_id*3+1] = tau(1);
    _joint_cmd-> tau[leg_id*3+2] = tau(2);
    
}

void IORos::SetZeroTau(int legID){
    _joint_cmd-> tau[legID*3+0] = 0;
    _joint_cmd-> tau[legID*3+1] = 0;
    _joint_cmd-> tau[legID*3+2] = 0;
}

void IORos::SetZeroTau(){
    for(uint8_t i=0;i<4;i++)
    {
        _joint_cmd-> tau[i*3+0] = 0;
        _joint_cmd-> tau[i*3+1] = 0;
        _joint_cmd-> tau[i*3+2] = 0;
    }
    
}

void IORos::SetZeroDq(int legID){
    _joint_cmd-> dq[legID*3+0] = 0;
    _joint_cmd-> dq[legID*3+1] = 0;
    _joint_cmd-> dq[legID*3+2] = 0;
}

void IORos::SetZeroDq(){
    for(int i(0); i<4; ++i){
        SetZeroDq(i);
    }
}

void IORos::SetFree()
{
    SetZeroDq();
    SetZeroTau();
}
void IORos::setStableGain(int legID){
    if(legID<2){
        _joint_cmd->kp[legID*3+0] = 18;
        _joint_cmd->kd[legID*3+0] = 2.8;
        _joint_cmd->kp[legID*3+1] = 28;
        _joint_cmd->kd[legID*3+1] = 3.2;
        _joint_cmd->kp[legID*3+2] = 28;
        _joint_cmd->kd[legID*3+2] = 3.2;
    }
    else{
        _joint_cmd->kp[legID*3+0] = 23;
        _joint_cmd->kd[legID*3+0] = 2.8;
        _joint_cmd->kp[legID*3+1] = 40;
        _joint_cmd->kd[legID*3+1] = 3.2;
        _joint_cmd->kp[legID*3+2] = 40;
        _joint_cmd->kd[legID*3+2] = 3.2;
    }
    
}
void IORos::setStableGain(){
    for(int i(0); i<4; ++i){
        setStableGain(i);
    }
}
void IORos::setSwingGain(int legID){
    if(legID<2){
        _joint_cmd->kp[legID*3+0] = 16;
        _joint_cmd->kd[legID*3+0] = 1.4;
        _joint_cmd->kp[legID*3+1] = 22;
        _joint_cmd->kd[legID*3+1] = 1.8;
        _joint_cmd->kp[legID*3+2] = 22;
        _joint_cmd->kd[legID*3+2] = 1.8;
    }
    else{
        _joint_cmd->kp[legID*3+0] = 20;
        _joint_cmd->kd[legID*3+0] = 1.4;
        _joint_cmd->kp[legID*3+1] = 28;
        _joint_cmd->kd[legID*3+1] = 1.8;
        _joint_cmd->kp[legID*3+2] = 28;
        _joint_cmd->kd[legID*3+2] = 1.8;
    }
    

}
IORos::~IORos(){
    rclcpp::shutdown();
}
