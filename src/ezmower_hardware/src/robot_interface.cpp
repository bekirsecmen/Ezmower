#include"ezmower_hardware/robot_interface.hpp"
#include<math.h>
#include"rclcpp/rclcpp.hpp"
#include"tf2_geometry_msgs/tf2_geometry_msgs.hpp"


robot_interface::robot_interface(int enc_counts_per_rev, int port_block_timeout_ms)
:encoder_counts_per_rev(enc_counts_per_rev)
{

    this->port_block_timeout_ms = port_block_timeout_ms;
    last_velocity_sample_time = std::chrono::system_clock::now();
     imu_sample_linear_acc_x = 0;
     imu_sample_linear_acc_y = 0;
     imu_sample_linear_acc_z = 0;
     imu_sample_orientation_roll = 0; //  x roll, 
     imu_sample_orientation_pitch = 0; // y pitch,
     imu_sample_orientation_yaw = 0; //  z yaw
     imu_sample_orientation_x = 0; // in quarternion;
     imu_sample_orientation_y = 0; // in quarternion;
     imu_sample_orientation_z  = 0; // in quarternion;
     imu_sample_orientation_w = 0; // in quarternion;
     imu_sample_angular_vel_x = 0; 
     imu_sample_angular_vel_y = 0; 
     imu_sample_angular_vel_z = 0; 
    
}

bool robot_interface::set_left_cps_value(esp_serial_driver &driver, float cps_left)
{
    esp_packets::data_frame_command_cps cps_data = {
        .cps_left = cps_left,
        .cps_right = 0,
    };
    auto packet = esp_packets::construct_data_packet(esp_packets::message_types::esp_speed_set_left,
    &cps_data,
    sizeof(esp_packets::data_frame_command_cps));
    bool is_ok = driver.send_data(&packet,sizeof(esp_packets::data_packet_generic),port_block_timeout_ms);
    return is_ok;
}


bool robot_interface::set_right_cps_value(esp_serial_driver &driver, float right)
{
    esp_packets::data_frame_command_cps cps_data = {
        .cps_left = 0,
        .cps_right = right,
    };
    auto packet = esp_packets::construct_data_packet(esp_packets::message_types::esp_speed_set_right,
    &cps_data,
    sizeof(esp_packets::data_frame_command_cps));
    bool is_ok = driver.send_data(&packet,sizeof(esp_packets::data_packet_generic),port_block_timeout_ms);
    return is_ok;
}

bool robot_interface::set_both_cps_values(esp_serial_driver &driver, float cps_left, float cps_right)
{
    esp_packets::data_frame_command_cps cps_data = {
        .cps_left = cps_left,
        .cps_right = cps_right,
    };
    auto packet = esp_packets::construct_data_packet(esp_packets::message_types::esp_speed_set_both,
    &cps_data,
    sizeof(esp_packets::data_frame_command_cps));
    bool is_ok = driver.send_data(&packet,sizeof(esp_packets::data_packet_generic),port_block_timeout_ms);
    return is_ok;
}

bool robot_interface::set_pid_values(esp_serial_driver &driver, float p, float i, float d)
{
    esp_packets::data_frame_command_pid dframe = {
        .kp = p,
        .ki = i,
        .kd = d,
    };

    auto packet = esp_packets::construct_data_packet(esp_packets::message_types::esp_set_pid,
    &dframe,
    sizeof(esp_packets::data_frame_command_pid));
    bool is_ok = driver.send_data(&packet,sizeof(esp_packets::data_packet_generic),port_block_timeout_ms);
    return is_ok;
}


void robot_interface::parse_packets_to_data_frames(esp_serial_driver& driver){
    // pop and proc packets in a loop 
    while(!driver.is_packet_queue_empty()){ 
        esp_packets::data_packet_generic packet;
        try
        {
            packet = driver.pop_packet();
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("ezmower_hardware"),"Attempt to read but no packets.");
            return;
        }
        

        esp_packets::message_types current_type = static_cast<esp_packets::message_types>(packet.message_type);
        // dispatch action to be taken based on the current frame type
        switch (current_type)
        {
            case esp_packets::message_types::esp_encoder_counts:
            {


                esp_packets::data_frame_encoder enc_counts_frame = {0};
                std::memcpy(&enc_counts_frame,packet.data,sizeof(esp_packets::data_frame_encoder));

                left_encoder_counts_rad = ( static_cast<double>(enc_counts_frame.encoder_count_left)) * (1 / static_cast<double>(encoder_counts_per_rev)) * (2 * M_PI);
                right_encoder_counts_rad = ( static_cast<double>(enc_counts_frame.encoder_count_right)) * (1 / static_cast<double>(encoder_counts_per_rev)) * (2 * M_PI);

                break;
            }

            case esp_packets::message_types::esp_imu_rot:
            {
                esp_packets::data_frame_imu_rot rot_frame = {0};
                std::memcpy(&rot_frame,packet.data,sizeof(esp_packets::data_frame_imu_rot));
                imu_sample_orientation_roll = rot_frame.roll * (M_PI /  180);
                imu_sample_orientation_pitch = rot_frame.pitch * (M_PI /  180);
                imu_sample_orientation_yaw = rot_frame.yaw * (M_PI /  180)  * -1;
                tf2::Quaternion quat;
                quat.setRPY(imu_sample_orientation_roll,imu_sample_orientation_pitch,imu_sample_orientation_yaw);
                imu_sample_orientation_x = quat.getX();
                imu_sample_orientation_y = quat.getY();
                imu_sample_orientation_z = quat.getZ();
                imu_sample_orientation_w = quat.getW();        

                // also get the angular speed
                imu_sample_angular_vel_x =  rot_frame.m_field_x * (M_PI /  180);        
                imu_sample_angular_vel_y =  rot_frame.m_field_z * (M_PI /  180);  
                imu_sample_angular_vel_z =  rot_frame.m_field_y * (M_PI /  180);  
                break;
            }
            case esp_packets::message_types::esp_imu_trans:
            {
                esp_packets::data_frame_imu_trans trans_packet = {0};
                std::memcpy(&trans_packet,packet.data,sizeof(esp_packets::data_frame_imu_trans));
                imu_sample_linear_acc_x = trans_packet.linear_acc_x;
                imu_sample_linear_acc_y = trans_packet.linear_acc_z;
                imu_sample_linear_acc_z = trans_packet.linear_acc_y;
                break;
            }
            case esp_packets::message_types::esp_imu_temp:
            {
                esp_packets::data_frame_imu_temp temp_packet = {0};
                std::memcpy(&temp_packet,packet.data,sizeof(esp_packets::data_frame_imu_temp));
                imu_sample_temp = temp_packet.temperature;
                break;
            }
            case esp_packets::message_types::esp_sonar_dist:
            {
                esp_packets::data_frame_sonar_dist sonar_frame = {0};
                std::memcpy(&sonar_frame,packet.data,sizeof(esp_packets::data_frame_sonar_dist));
                range_sensor_left_m = sonar_frame.distance_left / 100;
                range_sensor_center_m = sonar_frame.distance_center / 100;
                range_sensor_right_m = sonar_frame.distance_right / 100;
                break;
            }
   
                               
            default:
            {
                std::cout << "Non recognized packet came with packet type: " << packet.message_type << std::endl;
                break;
            }
            
        }
    }
}
