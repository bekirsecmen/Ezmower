#include <iostream>
#include <vector>
#include <memory> // For std::unique_ptr
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include"serial_driver.hpp"
#include"robot_interface.hpp"
#include <thread>
#include <chrono>

constexpr int CHUNK_SIZE = 100;   // 1KB read chunk size
constexpr int BUFFER_SIZE = 10 * CHUNK_SIZE; // 10KB buffer size

constexpr int PACKET_SIZE_TO_READ = 28;
constexpr int PACKET_WINDOW_SIZE = 4; // number of packets for reciever queue

static std::unique_ptr<esp_serial_driver> driver_ptr;
static std::unique_ptr<robot_interface> robot_interface_ptr;



int main() {
    try {

        driver_ptr = std::make_unique<esp_serial_driver>(BUFFER_SIZE,115200,"/dev/ttyUSB0",PACKET_SIZE_TO_READ);
        robot_interface_ptr = std::make_unique<robot_interface>();

        bool is_connected = driver_ptr->connect();
        while (true)
        {
            robot_interface_ptr->set_both_cps_values(*driver_ptr, 0,0);
            std::cout << "Loop spinning...." << std::endl;
            bool did_recieve = driver_ptr->recieve_some_data();
            robot_interface_ptr->parse_packets_to_data_frames(*driver_ptr);
            // test if we have could read the port 
            if(!did_recieve || !is_connected){
                // reattempt connection if failed
                while (!driver_ptr->is_connected())
                {
                    std::cout << "Device not available. Attempting to reconnect..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    driver_ptr->disconnect();
                    driver_ptr->connect();
                    is_connected = true;
                }
            } 
            else
            {
                // we acutally read some bytes, lets do further processing     
                std::cout << "Left encoder count : " << robot_interface_ptr->left_encoder_counts << std::endl;
                std::cout << "Right encoder count : " << robot_interface_ptr->right_encoder_counts << std::endl;
                std::cout << "IMU linear acc x : " << robot_interface_ptr->imu_sample_linear_acc_x << std::endl;
                std::cout << "IMU linear acc y : " << robot_interface_ptr->imu_sample_linear_acc_y << std::endl;
                std::cout << "IMU linear acc z : " << robot_interface_ptr->imu_sample_linear_acc_z << std::endl;
        
                std::cout << "IMU oreintation x : " << robot_interface_ptr->imu_sample_orientation_x << std::endl;
                std::cout << "IMU oreintation y : " << robot_interface_ptr->imu_sample_orientation_y << std::endl;
                std::cout << "IMU oreintation z : " << robot_interface_ptr->imu_sample_orientation_z << std::endl;

                std::cout << "IMU temperature reading: " << robot_interface_ptr->imu_sample_temp << std::endl;

                std::cout << "Range sensor left(m) : " << robot_interface_ptr->range_sensor_left_m << std::endl; 
                std::cout << "Range sensor center(m) : " << robot_interface_ptr->range_sensor_center_m << std::endl;
                std::cout << "Range sensor right(m) : " << robot_interface_ptr->range_sensor_right_m << std::endl;

            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        }
        
        

    } catch (std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
    }

    return 0;
}
