#ifndef SERIAL_PORT_HPP_
#define SERIAL_PORT_HPP_

#include <sys/types.h> // ssize_t

#include <string>
#include <functional>
#include <thread>

/// @brief callback signature for serial port successful connection event
using connect_callback = std::function<void()>;

/// @brief callback signature for data read from serial port
using read_callback = std::function<void(const char*, const size_t)>;

/// @brief class to read and write to serial port
class serial_port {
public:
    /// @brief serial_port constructor
    /// @param [in] portname Name which in Linux is a pseudo filename
    /// @param [in] baudrate Rate which must match the rate set on remote end
    /// @param [in] connect_function callback function called when port connects
    /// @param [in] read_function callback function called when data read
    /// @param [in] buffersize optional size of read buffer
    serial_port(const std::string& portname,
                int baudrate,
                connect_callback connect_function,
                read_callback read_function,
                size_t buffersize = 512);

    /// @brief destructor - clean up any resources
    ~serial_port();

    /// @brief open serial port
    /// @return new file descriptor or -ve Linux error code.
    int open();

    /// @brief write a single character to the serial port
    /// @param [in] ch - character to write
    /// @return 1 on success, or -1 on failure. Use errno for more details.
    ssize_t write(char ch);

    /// @brief write an array of characters to the serial port
    /// @param [in] data - character array to write
    /// @param [in] length of character array to write to port
    /// @return no. chars written on success, or -1 on failure. Use errno for more details.
    ssize_t write(const char* data, size_t length);

    /// @brief close serial port
    /// @return zero on success or -1 to indicate error. Use errno for more details.
    int close();

    /// @brief determine if serial port has been opened
    /// @return true if serial port already open
    bool is_open() const;

    /// @brief flush serial output
    /// @return 0 indicating successful flush or -1 for error. check errno for error.
    int flush();

    /// @brief get port name
    /// @return port string, example /dev/ttyACM0
    std::string get_portname() const {return m_port;}

private:
    /// @brief serial port identifier, example: /dev/ttyACM0
    std::string m_port;

    /// @brief baud rate, example 9600 or 115200
    int m_baudrate;

    /// @brief user defined connected event callback function
    connect_callback m_connect_function;
    /// @brief user defined data received callback function
    read_callback m_read_function;

    /// @brief size of read buffer
    size_t m_buffersize;

    /// @brief true if port is in process of being closed or is closed
    bool m_closing_port;

    /// @brief file descriptor, a unique identifier for port.
    /// -1 indicates an error or port not opened
    int m_fd = -1;

    /// @brief thread object used to run read_data() in separate thread
    std::thread m_reader;

    /// @brief read_data function which reads data and runs in separate thread
    void read_data();
};

#endif // SERIAL_PORT_HPP_
