#include "unitree_hardware_interface.hpp"

namespace unitree_hardware{

hardware_interface::CallbackReturn
    UnitreeHardwareInterface::on_init(const hardware_interface::HardwareInfo & info){
    if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    std::vector<std::pair<std::string, int>> config_map = {
        {"fr_hip_joint",   0},
        {"fr_knee_joint", 1},
        {"fr_thigh_joint",  2},
        {"fl_hip_joint",   3},
        {"fl_knee_joint", 4},
        {"fl_thigh_joint",  5},
        {"br_hip_joint",   6},
        {"br_knee_joint", 7},
        {"br_thigh_joint",  8},
        {"bl_hip_joint",   9},
        {"bl_knee_joint", 10},
        {"bl_thigh_joint",  11}
    };
     for (const auto & item  : config_map) {
        const std::string & name = item.first;
        int id = item.second;

        joint_name_to_id_[name] = id;
        joint_id_to_name_[id] = name;

        joint_names_.push_back(name);
    }
    

    hw_positions_.fill(0.0);
    hw_velocities_.fill(0.0);
    hw_efforts_.fill(0.0);
    hw_cmd_positions_.fill(0.0);
    hw_cmd_velocities_.fill(0.0);
    hw_cmd_efforts_.fill(0.0);
    hw_cmd_kps_.fill(0.0);
    hw_cmd_kds_.fill(0.0);
    for(int i=0;i<12;i++){
        _motor_cmd[i].motorType= MotorType::GO_M8010_6;
        _motor_cmd[i].mode= queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
        _motor_cmd[i].id   = i;
        _motor_cmd[i].kp = 0.0f;
        _motor_cmd[i].kd = 0.0f;
        _motor_cmd[i].q = 0.0f;
        _motor_cmd[i].dq = 0.0f;
        _motor_cmd[i].tau = 0.0f;

        _motor_data[i].motorType = MotorType::GO_M8010_6;
        _motor_data[i].q = 0.0f;
        _motor_data[i].dq = 0.0f;
        _motor_data[i].tau = 0.0f;
        // for(const auto & interface : info_.joints[i].command_interfaces){
        //     if (interface.name == "position"){
        //         if(interface.initial_value.empty()){
        //          std::cout<< "id: "<<i<<" iniq: empty "<<std::endl;
        //         }
        //         else{
        //             initial_q[i] = std::stod(interface.initial_value);
        //             std::cout<< "id: "<<i<<" iniq: "<<initial_q[i]<<std::endl;
        //         }
                
        //     }
        // }
        std::string current_joint_name = joint_id_to_name_[i];
        for(const auto & joint : info_.joints){
            if(joint.name == current_joint_name) {
                // 找到了，读取 initial_value
                for(const auto & interface : joint.command_interfaces){
                    if (interface.name == "position"){
                        if(!interface.initial_value.empty()){
                            initial_q[i] = std::stod(interface.initial_value);
                        }
                    }
                }
                break; // 找到了就退出内层循环
            }
        }
        std::cout<< "id: "<<i<<" iniq: "<<initial_q[i]<<std::endl;
    }
    _serial= new SerialPort("/dev/unitree485");

    for(int i=0;i<12;i++){
        if(_serial->sendRecv(&_motor_cmd[i],&_motor_data[i])){
            start_q[i] = _motor_data[i].q/6.33;

        }
        else
            start_q[i] = 0;
        std::cout<<"start_q: "<< start_q[i]<<std::endl;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
    UnitreeHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state){
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
    
}

hardware_interface::CallbackReturn
    UnitreeHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
        (void)previous_state;

        for(int i=0;i<12;i++){
        _motor_cmd[i].kp   = 0;
        _motor_cmd[i].kd   = 0.01;
        _motor_cmd[i].dq   = 0;
        _motor_cmd[i].tau  = 0;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}   

hardware_interface::CallbackReturn
    UnitreeHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    (void)previous_state;

    for(int i=0;i<12;i++){
    _motor_cmd[i].kp   = 0;
    _motor_cmd[i].kd   = 0;
    _motor_cmd[i].dq   = 0;
    _motor_cmd[i].tau  = 0;
    _serial->sendRecv(&_motor_cmd[i],&_motor_data[i]);
    }
    delete _serial;
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
    UnitreeHardwareInterface::export_state_interfaces() {

    std::vector<hardware_interface::StateInterface> interfaces;
    for (int i = 0; i < 12; i++) {
        const std::string & joint_name = joint_names_[i];
        interfaces.emplace_back(hardware_interface::StateInterface(joint_name,"effort",&hw_efforts_[i]));
        interfaces.emplace_back(hardware_interface::StateInterface(joint_name,"position",&hw_positions_[i]));
        interfaces.emplace_back(hardware_interface::StateInterface(joint_name,"velocity",&hw_velocities_[i]));
    }
    return interfaces;
}


std::vector<hardware_interface::CommandInterface>
UnitreeHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;

    for (int i = 0; i < 12; i++) {
        const std::string & joint_name = joint_names_[i];
        interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, "effort", &hw_cmd_efforts_[i]));
        interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, "position", &hw_cmd_positions_[i]));
        interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, "velocity", &hw_cmd_velocities_[i]));
        interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, "kp", &hw_cmd_kps_[i]));
        interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, "kd", &hw_cmd_kds_[i]));
    }

    return interfaces;
}

hardware_interface::return_type
    UnitreeHardwareInterface::read(const rclcpp::Time & time,const rclcpp::Duration & period){
    (void)time;
    (void)period;
    for (int i = 0; i < 12; i++) {
        std::string name = joint_names_[i];
        int motor_id = joint_name_to_id_.at(name); 

        switch (motor_id)
        {
        case 0:
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]+(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_velocities_[i] = static_cast<double>(_motor_data[motor_id].dq/6.33);
            hw_efforts_[i]    = static_cast<double>(_motor_data[motor_id].tau);
            break;
        case 1:
            hw_velocities_[i] = static_cast<double>(-_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]-(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(-_motor_data[motor_id].tau);
            break;
        case 2:
            hw_velocities_[i] = static_cast<double>(_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]+(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(_motor_data[motor_id].tau);
            break;
        case 3:
            hw_velocities_[i] = static_cast<double>(_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]+(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(_motor_data[motor_id].tau);
            break;
        case 4:
            hw_velocities_[i] = static_cast<double>(_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]+(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(_motor_data[motor_id].tau);
            break;
        case 5:
            hw_velocities_[i] = static_cast<double>(-_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]-(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(-_motor_data[motor_id].tau);
            break;
        case 6:
            hw_velocities_[i] = static_cast<double>(-_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]-(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(-_motor_data[motor_id].tau);
            break;
        case 7:
            hw_velocities_[i] = static_cast<double>(-_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]-(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(-_motor_data[motor_id].tau);
            break;
        case 8:
            hw_velocities_[i] = static_cast<double>(_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]+(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(_motor_data[motor_id].tau);
            break;
        case 9:
            hw_velocities_[i] = static_cast<double>(-_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]-(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(-_motor_data[motor_id].tau);
            break;
        case 10:
            hw_velocities_[i] = static_cast<double>(_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]+(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(_motor_data[motor_id].tau);
            break;
        case 11:
            hw_velocities_[i] = static_cast<double>(-_motor_data[motor_id].dq/6.33);
            hw_positions_[i]  = static_cast<double>(initial_q[motor_id]-(_motor_data[motor_id].q/6.33 - start_q[motor_id]));
            hw_efforts_[i]    = static_cast<double>(-_motor_data[motor_id].tau);
            break;
        }
        
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
    UnitreeHardwareInterface::write(const rclcpp::Time & time,const rclcpp::Duration & period){
    (void)time;
    (void)period;
    for (int i = 0; i < 12; i++) {
        std::string name = joint_names_[i];
        int motor_id = joint_name_to_id_.at(name);

        _motor_cmd[motor_id].kp  = static_cast<float>(hw_cmd_kps_[i]);
        _motor_cmd[motor_id].kd  = static_cast<float>(hw_cmd_kds_[i]);
        switch (motor_id)
        {
        case 0:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]+(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(hw_cmd_efforts_[i]);  
            break;
        case 1:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]-(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(-hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(-hw_cmd_efforts_[i]);
            break;
        case 2:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]+(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(hw_cmd_efforts_[i]);
            break;
        case 3:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]+(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(hw_cmd_efforts_[i]);
            break;
        case 4:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]+(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(hw_cmd_efforts_[i]);
            break;
        case 5:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]-(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(-hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(-hw_cmd_efforts_[i]);
            break;
        case 6:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]-(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(-hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(-hw_cmd_efforts_[i]);
            break;
        case 7:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]-(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(-hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(-hw_cmd_efforts_[i]);
            break;
        case 8:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]+(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(hw_cmd_efforts_[i]);
            break;
        case 9:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]-(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(-hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(-hw_cmd_efforts_[i]);
            break;
        case 10:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]+(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(hw_cmd_efforts_[i]);
            break;
        case 11:
            _motor_cmd[motor_id].q   = static_cast<float>((start_q[motor_id]-(hw_cmd_positions_[i]-initial_q[motor_id]))*6.33);
            _motor_cmd[motor_id].dq  = static_cast<float>(-hw_cmd_velocities_[i]*6.33);
            _motor_cmd[motor_id].tau = static_cast<float>(-hw_cmd_efforts_[i]);
            break;
        }
        // std::cout<<"id: "<<i<<"   q: "<< _motor_cmd[motor_id].q<<std::endl;
    }
    for(int i=0;i<12;i++){
        _serial->sendRecv(&_motor_cmd[i],&_motor_data[i]);
    }
    return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_hardware::UnitreeHardwareInterface,hardware_interface::SystemInterface)