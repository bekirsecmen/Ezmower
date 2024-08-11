#include"ezmower_hardware/serial_driver.hpp"
#include <boost/system/error_code.hpp>
#include"rclcpp/rclcpp.hpp"

/*****************************************************
 * Static Helper Functions                           *
******************************************************/

/**
 * @brief delete all instances of a given substring from a string
 * @param str the string to mutate
 * @param substr the substring to find and delete
 * @returns void
*/
static void find_and_delete_substr_in_str(std::string& str, const std::string& substr){
    size_t pos = str.find(substr);
    if(pos != std::string::npos){
        str.erase(pos,substr.length());
    }
}


/**
 * @brief finds and deletes all ansi escape sequences
 * @param str the string to mutate
 * @returns none
*/
static void sanitize_log_string(std::string& str){
    // find and delete all ansi escapes
    const std::string ansi_error = "[0;31m";
    const std::string ansi_info = "[0;32m";
    const std::string ansi_warn = "[0;33m";
    const std::string ansi_reset = "[0m";
    find_and_delete_substr_in_str(str,ansi_error);
    find_and_delete_substr_in_str(str,ansi_info);
    find_and_delete_substr_in_str(str,ansi_warn);
    find_and_delete_substr_in_str(str,ansi_reset);

}

/*****************************************************
 * esp_serial_driver implementation                  *
******************************************************/



void esp_serial_driver::parse_log_strings()
{
    std::vector<std::string> raw_logs;
    // attempt to find all newline delimited possible logs
    while(true){
        // try to find a newline
        auto newline_itr = std::find(log_buffer->begin(),log_buffer->end(),'\n');
        // if it is found, break it from there and push it to the vector as a new string
        if(newline_itr != log_buffer->end()){
            std::string new_log(log_buffer->begin(),newline_itr);
            raw_logs.push_back(new_log);
            // erase the read bytes from the buffer
            log_buffer->erase(log_buffer->begin(),newline_itr + 1);
        } else{
            // if no more newlines are found, break
            break;
        }
    }
    // now the read logs must be classified as either warn error or info
    for(const std::string& log : raw_logs){
        // try to match the ansi escapes with std::search
        auto error_escape_itr_start = std::find_end(log.begin(),log.end(),ansi_error.begin(),ansi_error.end());
        auto warn_escape_itr_start = std::find_end(log.begin(),log.end(),ansi_warn.begin(),ansi_warn.end());
        auto info_escape_itr_start = std::find_end(log.begin(),log.end(),ansi_info.begin(),ansi_info.end());
    
        if(error_escape_itr_start != log.end()){
            // now search for an ansi reset sequence
            auto ansi_reset_itr_start = std::search(error_escape_itr_start,log.end(),ansi_reset.begin(),ansi_reset.end());
            if(ansi_reset_itr_start != log.end()){
                // we have a hit, construct new string and add it to our vec
                auto error_escape_itr_end = error_escape_itr_start + ansi_error.length();
                // construct a new string
                std::string log_err(error_escape_itr_end,ansi_reset_itr_start);
                // sanitize it so that any escapes that might have escaped are erased
                sanitize_log_string(log_err);
                error_logs.push(log_err);
            }
        }   
        if(warn_escape_itr_start != log.end()){
            // now search for an ansi reset sequence
            auto ansi_reset_itr_start = std::search(warn_escape_itr_start,log.end(),ansi_reset.begin(),ansi_reset.end());
            if(ansi_reset_itr_start != log.end()){
                // we have a hit, construct new string and add it to our vec
                auto warn_escape_itr_end = warn_escape_itr_start + ansi_warn.length();
                // construct a new string
                std::string log_warn(warn_escape_itr_end,ansi_reset_itr_start);
                // sanitize it so that any escapes that might have escaped are erased
                sanitize_log_string(log_warn);
                warning_logs.push(log_warn);
            }
        }   
        if(info_escape_itr_start != log.end()){
            // now search for an ansi reset sequence
            auto ansi_reset_itr_start = std::search(info_escape_itr_start,log.end(),ansi_reset.begin(),ansi_reset.end());
            if(ansi_reset_itr_start != log.end()){
                // we have a hit, construct new string and add it to our vec
                auto info_escape_itr_end = info_escape_itr_start + ansi_info.length();
                // construct a new string
                std::string log_info(info_escape_itr_end,ansi_reset_itr_start);
                // sanitize it so that any escapes that might have escaped are erased
                sanitize_log_string(log_info);
                info_logs.push(log_info);
            }
        }  
    }
}


void esp_serial_driver::parse_packets()
{
    // find all occurences of the delimiter in the ring buffer 
    while (true)
    {
        auto itr_packet_delim = std::search(packet_buffer->begin(),packet_buffer->end(),packet_delimiter.begin(),packet_delimiter.end());
        if(itr_packet_delim != packet_buffer->end()){
            // we found a delimiter, overwrite delimiter so that it is not found again
            std::fill(itr_packet_delim,itr_packet_delim + packet_delimiter.size(),'0');
            // move the iterator to the start of the packet
            auto itr_packet_start = itr_packet_delim - ( esp_packets::PACKET_LENGTH - packet_delimiter.size()); // no exception possibility due to ring buffer 
            auto itr_packet_end = itr_packet_delim + packet_delimiter.size();
            // check if the range actually exists 
            if(!(itr_packet_start >= packet_buffer->begin() && itr_packet_end <= packet_buffer->end() && itr_packet_start <= itr_packet_end)){
                std::string err_str = "Invalid packet arrived with out of buffer iterators.";
                system_log_error(err_str);
                continue;
            }
            esp_packets::data_packet_generic packet;
            std::array<char,esp_packets::PACKET_LENGTH> packet_char;
            // get the binary data into the packet
            std::copy(itr_packet_start,itr_packet_end,packet_char.begin()); // HERE CAUSES NLLPTR ACCESS, INVESTIGATE
            // move the binary data into the POD struct
            std::memcpy(&packet,packet_char.data(),esp_packets::PACKET_LENGTH);
            // sanity check the packet by reading the size
            if(packet.message_length != esp_packets::PACKET_LENGTH){
                std::stringstream err_ss;
                err_ss << "This packet has the size " << packet.message_length << " instead of the correct size" << esp_packets::PACKET_LENGTH;
                std::string err_str = err_ss.str();
                system_log_error(err_str);
                continue;;
            }
            // push the packet
            processed_packets.push(packet);
        }
        else{
            break;
        }
    }
    
}

esp_serial_driver::esp_serial_driver(size_t buffer_size, int baud_rate, std::string port_name, size_t packet_length)
    : buffer_size(buffer_size), port_name(port_name), baud_rate(baud_rate)
{
    // construct the circ buffers as unique ptrs
    packet_buffer = std::make_unique<boost::circular_buffer<char>>(buffer_size);
    log_buffer = std::make_unique<boost::circular_buffer<char>>(buffer_size);
 }

esp_serial_driver::~esp_serial_driver()
{
}

bool esp_serial_driver::connect()
{
    try
    {
        // disconnect first, to be sure
        disconnect();
        // Create all of the internal objects with the configurations
        io_ctx_ptr = std::make_unique<boost::asio::io_service>();
        serial_port_ptr = std::make_unique<boost::asio::serial_port>(*io_ctx_ptr,port_name);
        serial_port_ptr->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_port_ptr->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_ptr->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_ptr->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_ptr->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        // test the connection and return
        return is_connected();
    }
    catch(const std::exception& e)
    {
        return false;
    }

}


bool esp_serial_driver::disconnect()
{
    try
    {
        // delete the old internal port objects so that they automatically close 
        serial_port_ptr.reset();
        io_ctx_ptr.reset();
        return true;
    }
    catch(const std::exception& e)
    {
        return false;
    }
    
}


bool esp_serial_driver::is_connected()
{
    // first check if the ptr itself is NULL
    if(!serial_port_ptr){
        return false;
    }
    // check if the serial port is open, this does not necessarily mean that it can be written to.
    bool connected = serial_port_ptr->is_open();
    if(connected){
        try
        {
            connected = false;
            auto test_byte = boost::asio::buffer("0");
            // write a dummy message to test the connection
            io_ctx_ptr->restart();
            serial_port_ptr->async_write_some(test_byte,
            [&connected, &test_byte](const boost::system::error_code ec, std::size_t bytes_transferred){
                if(!ec){
                    // if the error code is non-error, we have ok connection
                    connected = true;
                } 
                else{
                    connected = false;
                }
            });
            // run for timeout duration
            io_ctx_ptr->run_for(std::chrono::milliseconds(is_connected_timeout_ms));
            serial_port_ptr->cancel();
            io_ctx_ptr->reset();
        }
        catch(const std::exception& e)
        {
            connected = false;
        }
    }
    return connected;
}


bool esp_serial_driver::recieve_data(size_t data_size)
{
    std::vector<char> raw_data(data_size);
    try
    {
        if(!is_connected()){
            return false;
        }
        // receive the requested amount of bytes, block until this is satisified
        size_t bytes_read = boost::asio::read(*serial_port_ptr, boost::asio::buffer(raw_data), boost::asio::transfer_exactly(data_size));
        // insert the bytes read into the circular buffers for further processing
        packet_buffer->insert(packet_buffer->end(),raw_data.begin(), raw_data.begin() + bytes_read);
        log_buffer->insert(log_buffer->end(),raw_data.begin(), raw_data.begin() + bytes_read);
        // do the further processing
        parse_log_strings();
        parse_packets();
        return true;
    }
    catch(const std::exception& e)
    {
        std::string err = "Port read error, ";
        err += e.what();
        system_log_error(err);
        return false;
    }
}

bool esp_serial_driver::recieve_some_data()
{
    std::vector<char> raw_data(1024);
    try
    {
        if(!is_connected()){
            return false;
        }
        // recieve one or more bytes
        size_t bytes_read = serial_port_ptr->read_some(boost::asio::buffer(raw_data));
        // insert them into the circular buffers
        packet_buffer->insert(packet_buffer->end(),raw_data.begin(), raw_data.begin() + bytes_read);
        log_buffer->insert(log_buffer->end(),raw_data.begin(), raw_data.begin() + bytes_read);
        // process
        parse_log_strings();
        parse_packets();
        return true;
    }
    catch(const std::exception& e)
    {
        std::string err = "Port read error, ";
        err += e.what();
        system_log_error(err);
        return false;
    }
}


bool esp_serial_driver::recieve_some_data(int timeout_ms){
    std::vector<char> raw_data(1024);
    bool did_recieve = false;
    size_t bytes_read;
    try
    {
        // first test connection
        if(!is_connected()){
            std::string err = "Port read error, no connection.";
            system_log_error(err);
            return false;
        }       
        // restart the io context to flush out any unnecesary handlers
        io_ctx_ptr->restart();
        // queue the read operation with its callback 
        serial_port_ptr->async_read_some(boost::asio::buffer(raw_data),[&did_recieve,this,&bytes_read]
        (const boost::system::error_code ec, std::size_t bytes_transferred){
            if(!ec){
                did_recieve = true;
                bytes_read = bytes_transferred;
            }else{
                std::string err = "Port read error, ";
                err += ec.message();
                system_log_error(err);
                did_recieve = false;
            }
        });
        // run for the specified amount of time
        io_ctx_ptr->run_for(std::chrono::milliseconds(timeout_ms));
        // at this point, cancel and reset so that remaining handlers are flushed
        serial_port_ptr->cancel();
        io_ctx_ptr->reset();

        // process the recieved data if handler confirmed we recieved it 
        if(did_recieve){
            packet_buffer->insert(packet_buffer->end(),raw_data.begin(), raw_data.begin() + bytes_read);
            log_buffer->insert(log_buffer->end(),raw_data.begin(), raw_data.begin() + bytes_read);
            // process
            // std::string hard_debug(raw_data.begin(),raw_data.end());
            // RCLCPP_WARN(rclcpp::get_logger("ezmower_hardware"),hard_debug.c_str());

            parse_log_strings();
            parse_packets();
        }
    }
    catch(const std::exception& e)
    {
        std::string err = "Port read exception, ";
        err += e.what();
        system_log_error(err);
        return false;
    }
    return did_recieve;
}



bool esp_serial_driver::send_data(void *data, size_t size)
{
    if(!is_connected()){
        return false;
    }
    try
    {
        boost::asio::write(*serial_port_ptr, boost::asio::buffer(data, size));
        return true;
    }
    catch(const std::exception& e)
    {
        std::string err = "Port write error, ";
        err += e.what();
        system_log_error(err);
        return false;
    }
}



bool esp_serial_driver::send_data(void* data, size_t size, int timeout_ms){
    bool did_send = false;
    try
    {
        if(!is_connected()){
            std::string err = " Port write error, no connection";
            system_log_error(err);
            return false;
        }
        auto data_to_send = boost::asio::buffer(data,size);
        // write the data async
        io_ctx_ptr->restart();
        serial_port_ptr->async_write_some(data_to_send, 
        [this,&data_to_send,&did_send,size](const boost::system::error_code ec, std::size_t bytes_transferred){
            // test if we actualy sent it 
            if(!ec && bytes_transferred >= size){
                did_send = true;
            }
            else{ 
                did_send = false;
            }

        });

        // wait for the specified timeout for async operation to conclude
        io_ctx_ptr->run_for(std::chrono::milliseconds(timeout_ms));
        serial_port_ptr->cancel();
        io_ctx_ptr->reset();
    }
    catch(const std::exception& e)
    {
        std::string err = " Port write exception, ";
        err += e.what();
        system_log_error(err);
        return false;
    }
    return did_send;
}



std::string esp_serial_driver::pop_error_log()
{
    if(error_logs.empty()){
        throw std::out_of_range("Error logs are empty.");
    }
    else{
        std::string str;
        str = error_logs.front();
        error_logs.pop();
        return str;
    }
}

bool esp_serial_driver::is_error_log_emtpy()
{
    return error_logs.empty();
}

std::string esp_serial_driver::pop_warning_log()
{
    if(warning_logs.empty()){
        throw std::out_of_range("Warning logs are empty.");
    }
    else{

        std::string str;
        str = warning_logs.front();
        warning_logs.pop();
        return str;
    }
}

bool esp_serial_driver::is_warning_log_empty()
        {
            return warning_logs.empty();
        }

std::string esp_serial_driver::pop_info_log()
{
    if (info_logs.empty())
    {
        throw std::out_of_range("Info logs are empty.");
    }
    else
    {
        std::string str;
        str = info_logs.front();
        info_logs.pop();
        return str;
    }
}


bool esp_serial_driver::is_info_log_empty()
{
    return info_logs.empty();
}


esp_packets::data_packet_generic esp_serial_driver::pop_packet()
{
    if(processed_packets.empty()){
        throw std::out_of_range("Error logs are empty.");
    }
    else{
        esp_packets::data_packet_generic packet;
        packet = processed_packets.front();
        processed_packets.pop();
        return packet;
    }}

bool esp_serial_driver::is_packet_queue_empty(){
    return processed_packets.empty();
}

void esp_serial_driver::system_log_info(std::string & str)
{
    std::string log =  "[Serial Driver] INFO : " + str;
    RCLCPP_INFO(rclcpp::get_logger("ezmower_hardware"),log.c_str());
}   

void esp_serial_driver::system_log_warning(std::string & str)
{
    std::string log = "[Serial Driver] WARNING : " + str;
    RCLCPP_WARN(rclcpp::get_logger("ezmower_hardware"),log.c_str());
}

void esp_serial_driver::system_log_error(std::string & str)
{
    std::string log = "[Serial Driver] ERROR : " + str;
    RCLCPP_ERROR(rclcpp::get_logger("ezmower_hardware"),log.c_str());
}



/*****************************************************
 * esp_packets implementation                        *
******************************************************/


esp_packets::data_packet_generic esp_packets::construct_empty_packet(message_types type)
{
    // zero init the packet
    data_packet_generic packet = {0};
    // set termination
    std::fill(std::begin(packet.termination),std::end(packet.termination),'#');
    packet.message_length = PACKET_LENGTH;
    // set type
    packet.message_type = type;
    return packet;
}

esp_packets::data_packet_generic esp_packets::construct_data_packet(message_types type, void *data,size_t len)
{
    auto packet = esp_packets::construct_empty_packet(type);
    // populate the data 
    std::memcpy(packet.data,data,len);
    return packet;
}
