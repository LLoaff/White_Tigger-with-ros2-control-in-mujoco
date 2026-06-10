#ifndef IOROS_H
#define IOROS_H
// /pos_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory

#include "low/LowState.h"
#include "mathTypes.h"
#include "rclcpp/rclcpp.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
// #include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "motor_control_msgs/msg/dm_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "mathtool.h"
#include <csignal>
#include <unistd.h>
#include "geometry_msgs/msg/twist.hpp"
class IORos:public rclcpp::Node
{
public:
    IORos();
    ~IORos();
    double get_simulation_time() const {
    return now().seconds();
    }
    void upDate();
    void init();
    void stateCallBack(const sensor_msgs::msg::JointState& rec);
    void imuCallBack(const sensor_msgs::msg::Imu& rec);
    void cmdVelCallBack(const geometry_msgs::msg::Twist& rec);

    Eigen::Matrix<float,3,4> getQ();
    Eigen::Matrix<float,12,1> getQ12();
    Eigen::Matrix<float,12,1> getW12();

    void SetQ(Eigen::Matrix<float,12,1> q);
    void SetQ(int id,float q);
    void SetQ(int leg_id , Eigen::Matrix<float,3,1> q);
    void SetDq(int leg_id,Eigen::Matrix<float,3,1> dq);
    void SetDq(Eigen::Matrix<float,12,1> dq);
    void SetP(int leg_id,Eigen::Matrix<float,3,1> p);
    void SetP(Eigen::Matrix<float,12,1> p);
    void SetZeroP();
    void SetD(int leg_id,Eigen::Matrix<float,3,1> d);
    void SetD(Eigen::Matrix<float,12,1> d);
    void SetZeroD();
    void SetTau(Eigen::Matrix<float,12,1> tau,Eigen::Matrix<double,2,1> torqueLimit = Eigen::Matrix<double,2,1>(-9.0, 9.0));
    void SetTau(int leg_id,Eigen::Matrix<float,3,1> tau);
    void SetZeroTau(int legID);
    void SetZeroTau();
    void SetZeroDq(int legID);
    void SetZeroDq();
    void SetFree();

    void setStableGain(int legID);
    void setStableGain();
    void setSwingGain(int legID);

    LowState _state;

    Eigen::Matrix<float,12,1>   _init_q;
    float                       _cmd_vel[6];
    float vx_max;
    float vx_min;
    float vy_max;
    float vy_min;
    float wyaw_max;
    float wyaw_min;

private:
    rclcpp::Publisher<motor_control_msgs::msg::DmInterface >::SharedPtr _joint_puber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    std::shared_ptr<motor_control_msgs::msg::DmInterface> _joint_cmd;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
    const std::vector<std::string> _joint_order = {
    "fr_hip_joint", 
    "fr_thigh_joint",  
    "fr_calf_joint",  
    "fl_hip_joint",    
    "fl_thigh_joint",
    "fl_calf_joint",
    "br_hip_joint",
    "br_thigh_joint",
    "br_calf_joint",
    "bl_hip_joint",
    "bl_thigh_joint",
    "bl_calf_joint"
    };
    // std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> _joint_state;

    // MotorCmd _motor_cmd[12];
};

#endif


