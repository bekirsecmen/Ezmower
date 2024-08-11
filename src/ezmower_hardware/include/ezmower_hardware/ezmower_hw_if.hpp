#ifndef EZMOWER_HW_IF_HPP_
#define EZMOWER_HW_IF_HPP_
#include"rclcpp/rclcpp.hpp"
#include"hardware_interface/system_interface.hpp"
#include <memory> // For std::unique_ptr
#include"ezmower_hardware/robot_interface.hpp"
#include"ezmower_hardware/serial_driver.hpp"

using hardware_interface::return_type;

namespace ezmower_hardware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    struct hardware_config {
        std::string left_wheel_name = "left_wheel";
        std::string right_wheel_name = "right_wheel";
        std::string imu_sensor_name = "ezmower_imu";
        std::string sonar_sensor_name_left = "ezmower_range_left";
        std::string sonar_sensor_name_center = "ezmower_range_center";
        std::string sonar_sensor_name_right = "ezmower_range_right";
        float loop_rate = 30;
        std::string port_name = "/dev/ttyS0";
        int baud_rate = 115200;
        int encoder_counts_per_rev = 70;
        int port_block_timeout_ms = 150;
        int connection_timeout_ms = 5000;
        float kp = 0.6;
        float ki = 0.3;
        float kd = 0.05;
    };



    class ezmower_hw_if : public hardware_interface::SystemInterface
    {
    private:
    // The serial packet driver
    std::unique_ptr<esp_serial_driver> serial_driver_ptr;
    std::unique_ptr<robot_interface> robot_interface_ptr;
    hardware_config robot_config;

    double left_rad_per_s_wr;
    double right_rad_per_s_wr;  
    public:
        ezmower_hw_if(/* args */);
        bool reconnect_blocking();
        CallbackReturn on_configure( const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_cleanup( const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_shutdown( const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate( const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate( const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_error( const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    };
    
    

} // namespace ezmower_hardware


















#endif