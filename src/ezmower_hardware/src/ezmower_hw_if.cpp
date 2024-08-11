#include "ezmower_hardware/ezmower_hw_if.hpp"
#include <boost/asio.hpp>
#include<thread>
#include"hardware_interface/types/hardware_interface_type_values.hpp"
#include"hardware_interface/types/lifecycle_state_names.hpp"


ezmower_hardware::ezmower_hw_if::ezmower_hw_if()
{
    left_rad_per_s_wr = 0;
    right_rad_per_s_wr = 0;
}

bool ezmower_hardware::ezmower_hw_if::reconnect_blocking(){
    RCLCPP_WARN(rclcpp::get_logger("ezmower_hardware"), "connection down, trying to revive the connection...");
    int reconnect_delay_ms = 50;
    int reconnect_attempt_count = robot_config.connection_timeout_ms / reconnect_delay_ms;
;
    bool did_connect = false;
    for (size_t i = 0; i < reconnect_attempt_count; i++)
    {
        serial_driver_ptr->disconnect();
        serial_driver_ptr->connect();
        if(serial_driver_ptr->is_connected()){
            did_connect = true;
            RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Connection revived.");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
    }
    return did_connect;
}



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn ezmower_hardware::ezmower_hw_if::on_init(const hardware_interface::HardwareInfo& info){
    if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Starting initialization...");
    
    // get the configuration from ros 
    robot_config.left_wheel_name = info.hardware_parameters.at("left_wheel_name");
    robot_config.right_wheel_name = info.hardware_parameters.at("right_wheel_name");
    robot_config.imu_sensor_name = info.hardware_parameters.at("imu_sensor_name");
    robot_config.sonar_sensor_name_left = info.hardware_parameters.at("sonar_sensor_left_name");
    robot_config.sonar_sensor_name_center = info.hardware_parameters.at("sonar_sensor_center_name");
    robot_config.sonar_sensor_name_right = info.hardware_parameters.at("sonar_sensor_right_name");
    robot_config.port_block_timeout_ms = std::stof(info.hardware_parameters.at("port_block_timeout_ms"));
    robot_config.connection_timeout_ms = std::stof(info.hardware_parameters.at("connection_timeout_ms"));
    robot_config.loop_rate = std::stof(info.hardware_parameters.at("loop_rate"));
    robot_config.port_name = info.hardware_parameters.at("port_name");
    robot_config.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
    robot_config.encoder_counts_per_rev = std::stoi(info.hardware_parameters.at("enc_counts_per_rev"));
    robot_config.kp = std::stof(info.hardware_parameters.at("kp"));
    robot_config.ki = std::stof(info.hardware_parameters.at("ki"));
    robot_config.kd = std::stof(info.hardware_parameters.at("kd"));

    // allocate the objects 
    serial_driver_ptr = std::make_unique<esp_serial_driver>(5000,robot_config.baud_rate,robot_config.port_name,32);
    robot_interface_ptr = std::make_unique<robot_interface>(robot_config.encoder_counts_per_rev,robot_config.port_block_timeout_ms);
    if(serial_driver_ptr && robot_interface_ptr){
        RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"initializatied the ezmower hardware");
        return CallbackReturn::SUCCESS;
    } else{
        RCLCPP_ERROR(rclcpp::get_logger("ezmower_hardware"),"Cannot allocate driver.");
        return CallbackReturn::FAILURE;
    }

}


CallbackReturn ezmower_hardware::ezmower_hw_if::on_configure(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Starting configuration...");

    serial_driver_ptr->connect();
    if(serial_driver_ptr->is_connected()){
        RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Connected for the first time to ESP32.");
        esp_packets::data_frame_command_pid cmd = {
            .kp = robot_config.kp,
            .ki = robot_config.ki,
            .kd = robot_config.kd,
        };

        serial_driver_ptr->send_data(&cmd,sizeof(esp_packets::data_frame_command_pid),robot_config.connection_timeout_ms);
        RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Sent PID Params.");
        
        return CallbackReturn::SUCCESS;
    }
    else{
        return CallbackReturn::ERROR;
    }
}


CallbackReturn ezmower_hardware::ezmower_hw_if::on_cleanup(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Starting cleanup...");

    serial_driver_ptr->disconnect();
    return CallbackReturn::SUCCESS;
}


CallbackReturn ezmower_hardware::ezmower_hw_if::on_shutdown(const rclcpp_lifecycle::State &){
        RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Starting  shutdown...");

    return CallbackReturn::SUCCESS;
}


CallbackReturn ezmower_hardware::ezmower_hw_if::on_activate(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Starting activate...");

    return CallbackReturn::SUCCESS;    

}
CallbackReturn ezmower_hardware::ezmower_hw_if::on_deactivate(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Starting deactivate...");

    return CallbackReturn::SUCCESS;    
}
CallbackReturn ezmower_hardware::ezmower_hw_if::on_error(const rclcpp_lifecycle::State &){
    RCLCPP_ERROR(rclcpp::get_logger("ezmower_hardware"),"Timeout reached while trying to reconnect. HW_IF is dead.");    
    return CallbackReturn::FAILURE;
}

std::vector<hardware_interface::StateInterface> ezmower_hardware::ezmower_hw_if::export_state_interfaces(){
    RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Exporting state interfaces.");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.left_wheel_name,hardware_interface::HW_IF_POSITION,&(robot_interface_ptr->left_encoder_counts_rad)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.right_wheel_name,hardware_interface::HW_IF_POSITION,&(robot_interface_ptr->right_encoder_counts_rad)));


    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"orientation.x",&(robot_interface_ptr->imu_sample_orientation_x)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"orientation.y",&(robot_interface_ptr->imu_sample_orientation_y)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"orientation.z",&(robot_interface_ptr->imu_sample_orientation_z)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"orientation.w",&(robot_interface_ptr->imu_sample_orientation_w)));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"linear_acceleration.x",&(robot_interface_ptr->imu_sample_linear_acc_x)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"linear_acceleration.y",&(robot_interface_ptr->imu_sample_linear_acc_y)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"linear_acceleration.z",&(robot_interface_ptr->imu_sample_linear_acc_z)));

    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"angular_velocity.x",&(robot_interface_ptr->imu_sample_angular_vel_x)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"angular_velocity.y",&(robot_interface_ptr->imu_sample_angular_vel_y))); 
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.imu_sensor_name,"angular_velocity.z",&(robot_interface_ptr->imu_sample_angular_vel_z)));


    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.sonar_sensor_name_left,"range",&(robot_interface_ptr->range_sensor_left_m)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.sonar_sensor_name_center,"range",&(robot_interface_ptr->range_sensor_center_m)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(robot_config.sonar_sensor_name_right,"range",&(robot_interface_ptr->range_sensor_right_m)));
    return state_interfaces;

}

std::vector<hardware_interface::CommandInterface> ezmower_hardware::ezmower_hw_if::export_command_interfaces(){
    RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),"Exporting command interfaces.");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(robot_config.left_wheel_name,hardware_interface::HW_IF_VELOCITY,&(left_rad_per_s_wr)));    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(robot_config.right_wheel_name,hardware_interface::HW_IF_VELOCITY,&(right_rad_per_s_wr)));
    return command_interfaces;
}

return_type ezmower_hardware::ezmower_hw_if::read(const rclcpp::Time & time, const rclcpp::Duration & period){

    if(!serial_driver_ptr->is_connected()){
        RCLCPP_ERROR(rclcpp::get_logger("ezmower_hardware"),"cannot read, no connection .");
        bool final_attempt = reconnect_blocking();
        if(!final_attempt){
            return return_type::ERROR;
        }
    }

    
    bool did_recieve = serial_driver_ptr->recieve_some_data(robot_config.port_block_timeout_ms);
    if(!did_recieve){
        RCLCPP_WARN(rclcpp::get_logger("ezmower_hardware"),"Did not recieve any data.");
        bool final_attempt = reconnect_blocking();
        if(!final_attempt){
            return return_type::ERROR;
        }
    }

    robot_interface_ptr->parse_packets_to_data_frames(*serial_driver_ptr);
    //log the messages recieved from esp32
    // while(!serial_driver_ptr->is_info_log_empty()){
    //     std::string esp_info = "[ESP INFO] " +  serial_driver_ptr->pop_info_log();
    //     RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),esp_info.c_str());
    // }

    while(!serial_driver_ptr->is_warning_log_empty()){
        std::string esp_warn = "[ESP WARNING] " +  serial_driver_ptr->pop_warning_log();
        RCLCPP_WARN(rclcpp::get_logger("ezmower_hardware"),esp_warn.c_str());
    }
    
    while(!serial_driver_ptr->is_error_log_emtpy()){
        std::string esp_error = "[ESP ERROR] " + serial_driver_ptr->pop_error_log();
        RCLCPP_ERROR(rclcpp::get_logger("ezmower_hardware"),esp_error.c_str());
    }
    
    return return_type::OK;
}


return_type ezmower_hardware::ezmower_hw_if::write(const rclcpp::Time & time, const rclcpp::Duration & period){

    if(!serial_driver_ptr->is_connected()){
        bool final_attempt = reconnect_blocking();
        if(!final_attempt){
            return return_type::ERROR;
        }
    }

    float left_cps = left_rad_per_s_wr / ( 2* M_PI) * (robot_interface_ptr->encoder_counts_per_rev);
    float right_cps = right_rad_per_s_wr / ( 2* M_PI) * (robot_interface_ptr->encoder_counts_per_rev);
    bool did_send = robot_interface_ptr->set_both_cps_values(*serial_driver_ptr,left_cps,right_cps);
    if(!did_send){
        RCLCPP_WARN(rclcpp::get_logger("ezmower_hardware"),"Connection is up up but could not send data.");
    }


    
    return return_type::OK;
}







#include"pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ezmower_hardware::ezmower_hw_if,hardware_interface::SystemInterface)
