#include "serial_port.hpp"

#include <iostream>

// operating system includes
#include <unistd.h> // posix api, includes ssize_t, read, write, close
#include <fcntl.h> // file control operations, eg open
#include <cstdint>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/types.h> // ssize_t
#include <sys/ioctl.h> // TCGETS2
#include <string.h> // strerror

namespace
{
    termios2 get_termios2(int fd)
    {
        termios2 term2;
        int retcode = ioctl(fd, TCGETS2, &term2);
        if (retcode != 0)
        {
            std::cerr << "serial_port, ioctl(TCGETS2) error, errno: " << errno
                  << " " << strerror(errno) << std::endl;
        }

        return term2;
    }

    void set_termios2(termios2 tty, int fd)
    {
        int retcode = ioctl(fd, TCSETS2, &tty);
        if (retcode != 0)
        {
            std::cerr << "serial_port, ioctl(TCSETS2) error, errno: " << errno
                  << " " << strerror(errno) << std::endl;
        }
    }

    // configure tty
    void configure_terminal(int fd, int baudrate)
    {
        termios2 tty = get_termios2(fd);

        // .c_cflag
        // Set num. data bits
        tty.c_cflag &= ~CSIZE; // CSIZE is mask for number of bits per character
        tty.c_cflag |= CS8;

        // Set parity
        tty.c_cflag  &= ~PARENB;

        // Set stop bits
        tty.c_cflag &= ~CSTOPB;

        // Configure flow control - hardware flow control off
        tty.c_cflag &= ~CRTSCTS;

        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

        // baud rate
        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= CBAUDEX;
        tty.c_ispeed = baudrate;
        tty.c_ospeed = baudrate;

        // .c_oflag
        tty.c_oflag = 0;       // No remapping, no delays
        tty.c_oflag &= ~OPOST; // Make raw

        // Control characters .c_cc[]
        // Using select so no need for read timeouts
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;

        // .c_iflag
        // Configure software flow control off (XON/XOFF not interpreted as flow control)
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        // local modes c_lflag
        // In non-canonical mode, the rate at which read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
        tty.c_lflag &= ~ICANON;  // Turn off canonical input, which is suitable for pass-through
        // disable echo
        tty.c_lflag &= ~(ECHO);
        tty.c_lflag &= ~ECHOE;   // Turn off echo erase (echo erase only relevant if canonical input is active)
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;    // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

        set_termios2(tty, fd);
    }

} // anonymous namespace

serial_port::serial_port(const std::string& portname,
                         int baudrate,
                         connect_callback connect_function,
                         read_callback read_function,
                         size_t buffersize)
  : m_port(portname)
  , m_baudrate(baudrate)
  , m_connect_function(connect_function)
  , m_read_function(read_function)
  , m_buffersize(buffersize)
  , m_closing_port(false)
{
}

serial_port::~serial_port()
{
    if (is_open())
    {
        close();
    }
}

int serial_port::open()
{
    m_fd = ::open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (m_fd != -1)
    {
        configure_terminal(m_fd, m_baudrate);

        if(m_connect_function)
        {
            m_closing_port = false;
            m_connect_function();
        }

        m_reader = std::thread(&serial_port::read_data, this);
    }

    return m_fd;
}

int serial_port::close()
{
    int ret = 0;  // if already closed return success result code
    if(is_open())
    {
        m_closing_port = true;

        if (m_reader.joinable())
        {
            m_reader.join();
        }

        ret = ::close(m_fd);
        m_fd = -1;
    }
    return ret;
}

ssize_t serial_port::write(char ch)
{
    return ::write(m_fd, &ch, 1);
}

ssize_t serial_port::write(const char* data, size_t length)
{
    return ::write(m_fd, data, length);
}

bool serial_port::is_open() const
{
    return m_fd != -1;
}

int serial_port::flush()
{
    return is_open() ? ioctl(m_fd, TCSBRK, 1) : 0;
}

// runs in separate thread
void serial_port::read_data()
{
  std::cout << "read_data() started\n";
    fd_set readset;
    timeval  tv;

    // keep looping until close()
    while(!m_closing_port)
    {
        FD_ZERO(&readset); // clear the set
        FD_SET(m_fd, &readset); // add our file descriptor to the set

        // Wait up to response_timeout seconds for data
        tv.tv_sec = 1;  // select waits for up to 1 second before returning
        tv.tv_usec = 0;

        // wait for data to be read (with timeout)
        int rv = select(m_fd + 1, &readset, NULL, NULL, &tv);
        if(rv > 0)
        {
            // we have activity
            for (int i = m_fd; i < m_fd + 1; ++i)
            {
                if(FD_ISSET(m_fd, &readset))
                {
                    if(m_read_function)
                    {
                        char data[m_buffersize];
                        ssize_t bytes_read = ::read(m_fd, data, m_buffersize);
                        m_read_function(data, bytes_read);
                    }
                }
           }
        }
        else
        {
            // 0 means timeout, but -ve means select error
            if (rv < 0)
            {
                std::cerr << "serial_port, select returned: " << rv
                      << ", aborting reading port" << std::endl;
                m_closing_port = true;
            }
        }
    }  // while

    std::cout << "leaving read_data()\n";
}
