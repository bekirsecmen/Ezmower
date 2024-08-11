#pragma once

#include <iostream>
#include <vector>
#include <memory> // For std::unique_ptr
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <queue>
#include <mutex>




/**
 * Different packed structs for communucation and some utility functions used to communicate with the ESP32
*/
namespace esp_packets
{
    /**
     * This enum is encoded into the header of the packet to indicate the type of the data transmitted.
    */
    enum message_types {
        esp_command = 0, // Reserved for future use 
        esp_encoder_counts = 1, // Encoder count information
        esp_imu_trans = 2, // IMU translational acceleration information
        esp_imu_rot = 3, // IMU rotational orentation information
        esp_imu_temp = 4, // IMU temperature information
        esp_sonar_dist = 5, // Sonar distance information in cm, three channels with left center right
        esp_speed_set_left = 6, // Set speed command, expressed in counts per second, for the left wheel
        esp_speed_set_right = 7,// Set speed command, expressed in counts per second, for the right wheel
        esp_speed_set_both = 8, // Set speed command, expressed in counts per second, for both wheels
        esp_set_pid = 9, // set PID parameters command, which will set kp ki kd parameters
    };


    constexpr size_t PACKET_LENGTH = 32;
    // Generic data packet that is sent over serial.
    typedef struct __attribute__((packed)) {
        uint8_t message_length; // byte-wise length of the message, including termination characters
        uint8_t message_type; // type of the message, which will help parse it on the recieveing end
        uint8_t reserved1; 
        uint8_t reserved2; // First 4 bytes are always reserved for configuration
        char data[24]; // data payload that will vary with message_type
        char termination[4];  // termination so that the parsers can easily locate the packet
    } data_packet_generic;


    // Data frame that is used for transmitting encoder count information.
    typedef struct __attribute__((packed)) {
       int encoder_count_left;
       int encoder_count_right;
    } data_frame_encoder;

    // Data frame that is used for transmitting encoder PID values.
    typedef struct __attribute__((packed)) {
        float kp;
        float ki;
        float kd;
    } data_frame_command_pid;

    // Data frame that is used for set cps values.
    typedef struct __attribute__((packed)) {
        float cps_left;
        float cps_right;
    } data_frame_command_cps;

    // Data frame that is used for transmitting imu rotational information.
    typedef struct __attribute__((packed)) {
       float pitch;
       float yaw;
       float roll;
       float m_field_x;
       float m_field_y;
       float m_field_z;   
    } data_frame_imu_rot;

    // Data frame that is used for transmitting imu translational information.
    typedef struct __attribute__((packed)) {
       float linear_acc_x;
       float linear_acc_y;
       float linear_acc_z;
       float m_field_x;
       float m_field_y;
       float m_field_z;   
    } data_frame_imu_trans;

    // Data frame that is used for transmitting imu translational information.
    typedef struct __attribute__((packed)) {
       float temperature;
    } data_frame_imu_temp;

    // Data frame that is used for transmitting sonar distance information.
    typedef struct __attribute__((packed)) {
       float distance_left;
       float distance_center;
       float distance_right;
    } data_frame_sonar_dist;


    data_packet_generic construct_empty_packet(message_types type);
    data_packet_generic construct_data_packet(message_types type,void* data,size_t len);

} // namespace esp_ackets



/*
ESP 32 Serial RX TX driver
*/
class esp_serial_driver
{
private:
    // A circular buffer that holds the inbound chars, these chars are then parsed for packets
    std::unique_ptr<boost::circular_buffer<char>> packet_buffer;
    // A circular buffer that holds the inbound chars, these chars are then parsed for logs  
    std::unique_ptr<boost::circular_buffer<char>> log_buffer;
    // A queue that holds the processed packets
    std::queue<esp_packets::data_packet_generic> processed_packets;
    // error logs that came from the esp device
    std::queue<std::string> error_logs;
    // warning logs that came from the esp device
    std::queue<std::string> warning_logs;
    // info logs that came from the esp device
    std::queue<std::string> info_logs;

    // Port Settings
    
    // the size of the circular buffers that will hold the raw char data that is inbound over UART
    size_t buffer_size;
    // the name of the UART port that will be given to OS to connect 
    std::string port_name;
    // the BAUD rate of the UART port
    int baud_rate;
    // timeout before port is flagged as offline, in ms
    const int is_connected_timeout_ms = 500;

    // Port members

    // internal serial port object that is used for comms 
    std::unique_ptr<boost::asio::serial_port> serial_port_ptr;
    // internal io service executor that is needed by the port, we do not directly use this
    std::unique_ptr<boost::asio::io_service> io_ctx_ptr;

    // parser related constants

    // The ansi code for coloring terminal red, with the letter E, indicating ERROR
    const std::string ansi_error = "[0;31mE";
    // The ansi code for coloring terminal green, with the letter I, indicating INFO
    const std::string ansi_info = "[0;32mI";
    // The ansi code for coloring terminal yellow, with the letter W, indicating WARNING
    const std::string ansi_warn = "[0;33mW";
    // The ansi code for reseting terminal option such as color.
    const std::string ansi_reset = "[0m";
    // The packet termination sequence we look for during parsing packets
    const std::string packet_delimiter = "####";

    // internal parsing functions

    /**
     * Parses the char circular buffer for log strings and then deposits them into their respective std::queues.
    */
    void parse_log_strings();
 
    /**
     * Parses the char circular buffer for packets and then deposits them into their respective std::queues.
    */   
    void parse_packets();
public:
    /**
     * Constructs a serial driver object.
     * @param buffer_size the size of the internal buffers that will hold the incoming byte data.
     * @param baud_rate the baud rate of the communication over UART
     * @param port_name the name of the UART port given by the OS
     * @param packet_length The length of the packets that are inbound and outbound
    */
    esp_serial_driver(size_t buffer_size, int baud_rate, std::string port_name, size_t packet_length);
    ~esp_serial_driver();

    /**
     * @brief Attempts to connect to the serial port.
     * @returns bool, whether the connections attempt is successfull or not.
    */
    bool connect();

    /**
     * @brief Attempts to disconnect from the serial port.
     * @returns bool whether if the disconnect attempt is ok 
    */
    bool disconnect();
    /**
     * @brief Checks if the serial port is open, this will send a test byte to see this.
     * @return bool whether the serial port is open or not
    */
    bool is_connected();

    /**
     * Attempts to read bytes in a blocking fashion.
     * @param data_size how many data to get
     * @returns bool, if this is false this means that the port has died. call connect() again.
    */
    bool recieve_data(size_t data_size);

    /**
     * Recieves one or more bytes of data and deposits them into the internal buffer.
     * Will block until at least one byte is received. More bytes also may be recieved.
     * @return bool, if this is false this means that the port has died. call connect() again.
    */
    bool recieve_some_data();

    /**
     * Recieves one or more bytes of data in a blocking fashion and deposits them into the internal buffer.
     * Will block until at least one byte is received. More bytes also may be recieved.
     * @param timeout_ms timeout to exit blocking 
     * @return bool, if this is false this means that the port has died. call connect() again.
    */
    bool recieve_some_data(int timeout_ms);

    /**
     * Sends raw byte data over the UART channel.
     * @param data the void* that points to the data to be sent.
     * @param size the length of the raw data to be sent. 
     * @return bool whether the send was OK or not. 
    */
    bool send_data(void* data, size_t size);


    /**
     * Sends raw byte data over the UART channel.
     * @param data the void* that points to the data to be sent.
     * @param size the length of the raw data to be sent. 
     * @param timeout_ms timeout to exit blocking
     * @return bool whether the send was OK or not. 
    */
    bool send_data(void* data, size_t size, int timeout_ms);


    /**
     * Pops and returns an error from the esp device.
     * @returns the error string
     * @throws std::out_of_range if the error log queue is empty.
    */
    std::string pop_error_log();

    /**
     * Checks the error log queue that are recieved from the device.
     * @return True if the queue is empty, o/w false.
    */
    bool is_error_log_emtpy();

    /**
     * Pops and returns a warning from the esp device.
     * @returns the warning string
     * @throws std::out_of_range if the warning log queue is empty.
    */
    std::string pop_warning_log();

    /**
     * Checks the warning log queue that are recieved from the device.
     * @return True if the queue is empty, o/w false.
    */
    bool is_warning_log_empty();

    /**
     * Pops and returns an info log from the esp device.
     * @returns the info log string
     * @throws std::out_of_range if the info log queue is empty.
    */
    std::string pop_info_log();

    /**
     * Checks the info log queue that are recieved from the device.
     * @return True if the queue is empty, o/w false.
    */
    bool is_info_log_empty();

    /**
     * Pops and returns a valid packet from the proccessed packets queue.
     * @returns the packet 
     * @throws std::out_of_range if the queue is empty
    */
    esp_packets::data_packet_generic pop_packet();

    /**
     * Checks if the processed packets queue is empty.
     * @returns bool, True if the queue is empty. o/w false.
    */
    bool is_packet_queue_empty();

    /**
     * Virtual function that logs errors that happen on the system, not on the esp device.
     * This can be overriden by children to redirect logs.
     * @param str the log info string to be logged.
    */
    virtual void system_log_info(std::string& str);


    /**
     * Virtual function that logs errors that happen on the system, not on the esp device.
     * This can be overriden by children to redirect logs.
     * @param str the log warning string to be logged.
    */
    virtual void system_log_warning(std::string& str);

    /**
     * Virtual function that logs errors that happen on the system, not on the esp device.
     * This can be overriden by children to redirect logs.
     * @param str the log error string to be logged.
    */
    virtual void system_log_error(std::string& str);

};


