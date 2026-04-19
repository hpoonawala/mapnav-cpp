#ifndef SERIAL_WRITER_H
#define SERIAL_WRITER_H

#include <asio.hpp>
#include <asio/serial_port.hpp>
#include <string>

class SerialWriter {
    

public:
    asio::io_context io_context; 
    asio::serial_port serial_port;
    // Constructor - opens and configures serial port
    SerialWriter(const std::string& port_name, unsigned int baud_rate = 115200);
    
    // Destructor - automatically closes the port
    ~SerialWriter();
    
    // Write a message to the serial port
    void write(const std::string& message);
    
    // Check if port is open
    bool is_open() const;
};

// Utility function for one-off messages
void write_serial_message(const std::string& port_name, const std::string& message, unsigned int baud_rate = 115200);

std::string write_and_read_serial_message(const std::string&,
                                          const std::string&,
                                          unsigned int,
                                          unsigned int );
#endif // SERIAL_WRITER_H
