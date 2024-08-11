#pragma once

#include <iostream>
#include <vector>
#include <memory> // For std::unique_ptr
#include <queue>
#include <mutex>
#include<chrono>
#include"serial_driver.hpp"
/**
 * Robot interface class that implements interfaces so that the robot data can be read and written.
 * The instance gets packet data from esp_serial_driver objects and then writes them into its public member variables.
*/
class robot_interface{
public:
    int encoder_counts_per_rev;


    std::chrono::time_point<std::chrono::system_clock> last_velocity_sample_time;
    double imu_sample_linear_acc_x;
    double imu_sample_linear_acc_y;
    double imu_sample_linear_acc_z;
    double imu_sample_orientation_roll; //  x roll, 
    double imu_sample_orientation_pitch; // y pitch,
    double imu_sample_orientation_yaw; //  z yaw
    double imu_sample_orientation_x; // in quarternion;
    double imu_sample_orientation_y; // in quarternion;
    double imu_sample_orientation_z; // in quarternion;
    double imu_sample_orientation_w; // in quarternion;
    double imu_sample_angular_vel_x; 
    double imu_sample_angular_vel_y; 
    double imu_sample_angular_vel_z; 
       
    double imu_sample_unused = 0;


    double imu_sample_temp;
    double range_sensor_left_m;
    double range_sensor_center_m;
    double range_sensor_right_m;
    double left_encoder_counts_rad;
    double right_encoder_counts_rad;
    int port_block_timeout_ms;

    robot_interface(int enc_counts_per_rev, int port_block_timeout_ms);
    /**
     * Set the left count per second value of the robot.
     * @param driver the esp_serial_driver that will do the writing
     * @param cps_left the left count per seconds value that will be sent
     * @retunrs the success of the operation
    */
    bool set_left_cps_value(esp_serial_driver& driver, float cps_left);
    /**
     * Set the right count per second value of the robot.
     * @param driver the esp_serial_driver that will do the writing
     * @param cps_right the right count per seconds value that will be sent
     * @retunrs the success of the operation
    */
    bool set_right_cps_value(esp_serial_driver& driver, float right);
    /**
     * Set both count per second values of the robot.
     * @param driver the esp_serial_driver that will do the writing
     * @param cps_left the left count per seconds value that will be sent
     * @param cps_right the right count per seconds value that will be sent
     * @retunrs the success of the operation
    */
    bool set_both_cps_values(esp_serial_driver& driver, float cps_left,float cps_right);
    /**
     * Sets the PID parameters of the robot.
     * @param driver the driver that will do the writing
     * @param p kp
     * @param i ki
     * @param d kd
     * @returns the success of the operation
    */
    bool set_pid_values(esp_serial_driver& driver, float p,float i, float d);
    /**
     * This function will take the recieved packets from the serial driver and process them into 
     * data frames so that they can be later read from this classes public member variables. Call this
     * after you read some bytes from the serial device.
     * @param driver the driver from which the packets will be fetched
     * 
    */
    void parse_packets_to_data_frames(esp_serial_driver& driver);

};
