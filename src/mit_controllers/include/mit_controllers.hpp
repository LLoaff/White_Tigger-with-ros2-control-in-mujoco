#ifndef MIT_CONTROLLERS_HPP
#define MIT_CONTROLLERS_HPP

#include "controller_interface/controller_interface.hpp"
#include "motor_control_msgs/msg/dm_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>

namespace mit_controller{

class MitControllers : public controller_interface::ControllerInterface
{
public:
    MitControllers();
    // ~MitControllers();

    controller_interface::InterfaceConfiguration command_interface_configuration()                       const override;
    controller_interface::InterfaceConfiguration state_interface_configuration()                         const override;

    controller_interface::CallbackReturn on_init()                                                       override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)    override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)     override;

    controller_interface::return_type update(const rclcpp::Time & time , const rclcpp::Duration & period)override;
    
protected:
    std::vector<std::string> joint_names_;
    // std::unordered_map<std::string, size_t> name_map_;
    // std::unordered_map<int, std::string> id2name;

    struct MotorCommand {
        std::vector<double> q;    // 目标位置
        std::vector<double> dq;   // 目标速度
        std::vector<double> kp;   // 刚度
        std::vector<double> kd;   // 阻尼
        std::vector<double> tau;  // 前馈力矩
        bool initialized = false; // 标记是否收到了第一帧指令
    };
    MotorCommand latest_command_;
    std::mutex command_mutex_;

    rclcpp::Subscription<motor_control_msgs::msg::DmInterface>::SharedPtr command_subscriber_;
    rclcpp::Publisher<motor_control_msgs::msg::DmInterface>::SharedPtr    command_pub_;


    void commandCallback(const motor_control_msgs::msg::DmInterface::SharedPtr msg);
};
}// namespace mit_controller


#endif


