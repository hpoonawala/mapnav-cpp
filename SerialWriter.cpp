#include "SerialWriter.h"
#include <iostream>
#include <system_error>
#include <stdexcept>
#include <string>
#include <chrono>
#include <thread>

// Constructor - opens and configures serial port
SerialWriter::SerialWriter(const std::string& port_name, unsigned int baud_rate) 
    : serial_port(io_context) {
    
    try {
        // Open the serial port
        serial_port.open(port_name);
        
        // Configure serial port settings
        serial_port.set_option(asio::serial_port_base::baud_rate(baud_rate));
        serial_port.set_option(asio::serial_port_base::character_size(8));
        serial_port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serial_port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serial_port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
        
        //std::cout << "Serial port " << port_name << " opened at " << baud_rate << " baud" << std::endl;
        
    } catch (const std::system_error& e) {
        std::cerr << "Error opening serial port " << port_name << ": " << e.what() << std::endl;
        throw;
    }
}

// Destructor - automatically closes the port
SerialWriter::~SerialWriter() {
    if (serial_port.is_open()) {
        serial_port.close();
    }
}

// Write a message to the serial port
void SerialWriter::write(const std::string& message) {
    if (!serial_port.is_open()) {
        throw std::runtime_error("Serial port is not open");
    }
    
    try {
        asio::write(serial_port, asio::buffer(message));
        std::cout << "Sent: " << message << std::endl;
    } catch (const std::system_error& e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
        throw;
    }
}

// Check if port is open
bool SerialWriter::is_open() const {
    return serial_port.is_open();
}

// Simple function for one-off messages (equivalent to Python with statement)
void write_serial_message(const std::string& port_name, const std::string& message, unsigned int baud_rate) {
    try {
        SerialWriter writer(port_name, baud_rate);
        writer.write(message);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        throw;
    }
}

// Read and write serial message with response (equivalent to Python with statement)
std::string write_and_read_serial_message(const std::string& port_name,
                                          const std::string& message,
                                          unsigned int baud_rate,
                                          unsigned int timeout_ms = 1000) {
    try {
        SerialWriter writer(port_name, baud_rate);
        writer.write(message);

        // Wait a bit for response to arrive
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Read response
        asio::streambuf receive_buffer;
        asio::error_code ec;

        // Set up deadline timer for timeout
        asio::steady_timer timer(writer.io_context);
        timer.expires_after(std::chrono::milliseconds(timeout_ms));

        std::size_t bytes_read = 0;
        bool timeout_occurred = false;

        // Start async read
        asio::async_read_until(writer.serial_port, receive_buffer, '\n',
            [&](const asio::error_code& error, std::size_t bytes) {
                ec = error;
                bytes_read = bytes;
                timer.cancel();
            });

        // Start timer
        timer.async_wait([&](const asio::error_code& error) {
            if (!error) {
                timeout_occurred = true;
                writer.serial_port.cancel();
            }
        });

        // Run until one completes
        writer.io_context.run();
        writer.io_context.restart();

        if (timeout_occurred) {
            throw std::runtime_error("Timeout waiting for response");
        }

        if (ec && ec != asio::error::eof) {
            throw std::system_error(ec);
        }

        // Extract the response
        std::string response;
        std::istream is(&receive_buffer);
        std::getline(is, response);

        std::cout << "Received: " << response << std::endl;
        return response;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        throw;
    }
}
