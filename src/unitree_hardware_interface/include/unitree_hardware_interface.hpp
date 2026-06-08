#ifndef UNITREE_HARDWARE_INTERFACE_H
#define UNITREE_HARDWARE_INTERFACE_H

#include "unitreeMotor/unitreeMotor.h"
#include "hardware_interface/system_interface.hpp"
// #include "CSerialPort/SerialPort.h"
#include "serialPort/SerialPort.h"
#include <array>

namespace unitree_hardware{

class UnitreeHardwareInterface:public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time,const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time,const rclcpp::Duration & period) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    MotorData  _motor_data[12];
    MotorCmd   _motor_cmd[12];
    // itas109::CSerialPort      _serial;
    SerialPort*                  _serial;
    std::vector<std::string> joint_names_;

    std::array<double, 12> hw_positions_;
    std::array<double, 12> hw_velocities_;
    std::array<double, 12> hw_efforts_;

    std::array<double, 12> hw_cmd_positions_;
    std::array<double, 12> hw_cmd_velocities_;
    std::array<double, 12> hw_cmd_efforts_;
    std::array<double, 12> hw_cmd_kps_;
    std::array<double, 12> hw_cmd_kds_;

    float                  initial_q[12];
    float                  start_q[12];
    std::unordered_map<std::string, int> joint_name_to_id_;
    std::unordered_map<int, std::string> joint_id_to_name_;
};
}
#endif

