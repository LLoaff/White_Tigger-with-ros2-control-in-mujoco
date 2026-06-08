#include "unitree_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace unitree_controller{
MitControllers::MitControllers(): controller_interface::ControllerInterface(){
}

controller_interface::CallbackReturn MitControllers::on_init(){
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitControllers::on_configure(const rclcpp_lifecycle::State & previous_state){
    (void) previous_state;
    joint_names_ = get_node()->get_parameter("joints").as_string_array();

    // for (size_t i = 0; i < joint_names_.size(); ++i) {
    // RCLCPP_INFO(get_node()->get_logger(), "  [Index %zu] -> %s", i, joint_names_[i].c_str());
    // }
    // name_map_.clear();
    // for(size_t i = 0; i < joint_names_.size(); i++){
    //     name_map_[joint_names_[i]] = i;
    // }
    // id2name .clear();
    // id2name[0] = "fr_hip_joint";
    // id2name[1] = "fr_knee_joint";
    // id2name[2] = "fr_thigh_joint";
    // id2name[3] = "fl_hip_joint";
    // id2name[4] = "fl_knee_joint";
    // id2name[5] = "fl_thigh_joint";
    // id2name[6] = "br_hip_joint";
    // id2name[7] = "br_knee_joint";
    // id2name[8] = "br_thigh_joint";
    // id2name[9] = "bl_hip_joint";
    // id2name[10] = "bl_knee_joint";
    // id2name[11] = "bl_thigh_joint";

    const size_t num_joints = joint_names_.size();
    latest_command_.q.resize(num_joints, 0.0);
    latest_command_.dq.resize(num_joints, 0.0);
    latest_command_.kp.resize(num_joints, 0.0);
    latest_command_.kd.resize(num_joints, 0.0);
    latest_command_.tau.resize(num_joints, 0.0);

    command_pub_ = get_node()->create_publisher<motor_control_msgs::msg::DmInterface>(
        "/dm_state", 1
    );
    command_subscriber_ = get_node()->create_subscription<motor_control_msgs::msg::DmInterface>(
        "/dm_command", 1,
        std::bind(&MitControllers::commandCallback, this, std::placeholders::_1)
    );

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitControllers::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    (void) previous_state;
    // for (auto & command_interface : command_interfaces_) {
    //     command_interface.set_value(0.0);
    // }
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MitControllers::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT );
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.push_back(joint_name + "/" + "kp");
        config.names.push_back(joint_name + "/" + "kd");
    }
    return config;
}

controller_interface::InterfaceConfiguration MitControllers::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    
    // 遍历所有关节，申请 "joint_name/position" 和 "joint_name/velocity"
    for (const auto & joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

void MitControllers::commandCallback(const motor_control_msgs::msg::DmInterface::SharedPtr msg)
{


    std::lock_guard<std::mutex> lock(command_mutex_);
    rclcpp::Time now = get_node()->now();
    rclcpp::Time msg_time(msg->header.stamp);

    // auto delay = now - msg_time;
    // if(delay.nanoseconds() > 50000000) // 50ms
    // {
    //     return;
    // }

    if (msg->id.empty() || 
        msg->id.size() != msg->q.size() || 
        msg->id.size() != msg->dq.size() ||
        msg->id.size() != msg->kp.size() ||
        msg->id.size() != msg->kd.size() ||
        msg->id.size() != msg->tau.size()) {
        RCLCPP_WARN(get_node()->get_logger(), "指令格式错误!id 与数据数组长度不一致");
        return;
    }

    for (size_t i = 0; i < msg->id.size(); ++i) {
        size_t joint_idx = static_cast<size_t>(msg->id[i]);
     
        latest_command_.q[joint_idx]   = static_cast<double>(msg->q[i]);
        latest_command_.dq[joint_idx]  = static_cast<double>(msg->dq[i]);
        latest_command_.kp[joint_idx]  = static_cast<double>(msg->kp[i]);
        latest_command_.kd[joint_idx]  = static_cast<double>(msg->kd[i]);
        latest_command_.tau[joint_idx] = static_cast<double>(msg->tau[i]);
        // RCLCPP_INFO(get_node()->get_logger(), " index:%d  q: %f \n ", (int)joint_idx, (float)latest_command_.q[joint_idx]);

    }
    latest_command_.initialized = true;
}

controller_interface::return_type MitControllers::update(const rclcpp::Time & time, const rclcpp::Duration & period){
    (void )period;
    if (!latest_command_.initialized) {
        return controller_interface::return_type::OK;
    }

    const size_t num_joints = joint_names_.size();
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        for (size_t i = 0; i < num_joints; ++i) {

            command_interfaces_[i*5+0].set_value(latest_command_.tau[i]);
            command_interfaces_[i*5+1].set_value(latest_command_.q[i]);
            command_interfaces_[i*5+2].set_value(latest_command_.dq[i]);
            command_interfaces_[i*5+3].set_value(latest_command_.kp[i]);
            command_interfaces_[i*5+4].set_value(latest_command_.kd[i]);
        }

    }
    static auto last_pub_time = time;
    if ((time - last_pub_time).nanoseconds() >= 100000000) { // 0.1秒
        std::lock_guard<std::mutex> lock(command_mutex_);
        
        motor_control_msgs::msg::DmInterface msg;
        msg.header.stamp= time;
        msg.id.resize(num_joints);
        msg.q.resize(num_joints);
        msg.dq.resize(num_joints);
        msg.kp.resize(num_joints);
        msg.kd.resize(num_joints);
        msg.tau.resize(num_joints);
        
        for (size_t i = 0; i < num_joints; ++i) {
            msg.id[i] = static_cast<int64_t>(i);
            msg.tau[i]= static_cast<float>(state_interfaces_[3*i+0].get_value());
            msg.q[i]  = static_cast<float>(state_interfaces_[3*i+1].get_value());
            msg.dq[i] = static_cast<float>(state_interfaces_[3*i+2].get_value());
            
            msg.kp[i] = static_cast<float>(latest_command_.kp[i]);
            msg.kd[i] = static_cast<float>(latest_command_.kd[i]);
        }

        command_pub_->publish(msg);
        last_pub_time = time;
    }

    return controller_interface::return_type::OK;
}

}



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(unitree_controller::MitControllers,controller_interface::ControllerInterface)