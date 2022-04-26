#include "serialport.h"

#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace UT {

bool SerialPort::m_threadRunning = false;
int SerialPort::m_pipe[2] = { 0, 0 };
std::mutex SerialPort::m_interruptMutex;
std::mutex SerialPort::m_mutex;
std::mutex SerialPort::m_threadInstanceMutex;
std::thread *SerialPort::m_pollingThread = nullptr;
std::vector<SerialPort *> SerialPort::m_instances;
std::vector<pollfd> SerialPort::m_pollFds;

SerialPort::SerialPort()
: m_baudRate(BaudRate::k115200),
  m_dataBits(DataBits::k8),
  m_parity(Parity::kNone),
  m_stopBits(StopBits::kOne) {
    memset(&m_options, 0, sizeof(m_options));

    m_threadInstanceMutex.lock();
    if (!m_pollingThread) {
        // Create pipes
        auto ret = pipe(m_pipe);
        if (ret == -1) {
            throw std::runtime_error("pipe(...) failed");
        }

        // Insert null-instance and pipe polling fd
        m_pollFds.clear();
        m_instances.clear();

        m_pollFds.emplace_back(pollfd { m_pipe[0], POLLIN, 0 });
        m_instances.emplace_back(nullptr);

        // Start polling thread
        m_threadRunning = true;
        m_pollingThread = new std::thread(polling);
    }
    m_threadInstanceMutex.unlock();
}

SerialPort::~SerialPort() {
    close();

    std::unique_lock lock(m_threadInstanceMutex);
    if (m_pollFds.size()) {
        return;
    }

    // Send interrupt and delete thread
    m_threadRunning = false;
    char code = '0';
    ::write(m_pipe[1], &code, sizeof(code));
    m_pollingThread->join();
    delete m_pollingThread;
    m_pollingThread = nullptr;

    // Close pipes
    ::close(m_pipe[0]);
    ::close(m_pipe[1]);
}

SerialPort::RetCode SerialPort::open(const std::string &port) {
    if (m_fd != 0) {
        return RetCode::kAlreadyOpened;
    }

    m_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (m_fd == -1) {
        m_fd = 0;
        switch (errno) {
        case ENOENT:
            return RetCode::kDeviceNotConnected;
        default:
            return RetCode::kUndefinedError;
        }
    }

    // Clear I/O buffers
    auto ret = ioctl(m_fd, TCFLSH, 2);
    if (ret == -1) {
        ::close(m_fd);
        m_fd = 0;
        return RetCode::kBufferFlushError;
    }

    // Set descriptor to non-blocking mode
    ret = fcntl(m_fd, F_SETFL, FNDELAY);
    if (ret == -1) {
        ::close(m_fd);
        m_fd = 0;
        return RetCode::kSwitchToNonBlockingModeError;
    }

    // Get current serial port options
    ret = tcgetattr(m_fd, &m_options);
    if (ret == -1) {
        ::close(m_fd);
        m_fd = 0;
        return RetCode::kFailedToGetPortOptions;
    }

    // Set local mode and enable receiver
    m_options.c_cflag |= (CLOCAL | CREAD);

    // To raw mode
    cfmakeraw(&m_options);

    // Set speed (baud)
    switch (m_baudRate) {
    case BaudRate::k0:
        cfsetispeed(&m_options, B0);
        cfsetospeed(&m_options, B0);
        break;
        
    case BaudRate::k50:
        cfsetispeed(&m_options, B50);
        cfsetospeed(&m_options, B50);
        break;

    case BaudRate::k75:
        cfsetispeed(&m_options, B75);
        cfsetospeed(&m_options, B75);
        break;

    case BaudRate::k110:
        cfsetispeed(&m_options, B110);
        cfsetospeed(&m_options, B110);
        break;

    case BaudRate::k134:
        cfsetispeed(&m_options, B134);
        cfsetospeed(&m_options, B134);
        break;

    case BaudRate::k150:
        cfsetispeed(&m_options, B150);
        cfsetospeed(&m_options, B150);
        break;

    case BaudRate::k200:
        cfsetispeed(&m_options, B200);
        cfsetospeed(&m_options, B200);
        break;

    case BaudRate::k300:
        cfsetispeed(&m_options, B300);
        cfsetospeed(&m_options, B300);
        break;

    case BaudRate::k600:
        cfsetispeed(&m_options, B600);
        cfsetospeed(&m_options, B600);
        break;

    case BaudRate::k1200:
        cfsetispeed(&m_options, B1200);
        cfsetospeed(&m_options, B1200);
        break;

    case BaudRate::k1800:
        cfsetispeed(&m_options, B1800);
        cfsetospeed(&m_options, B1800);
        break;

    case BaudRate::k2400:
        cfsetispeed(&m_options, B2400);
        cfsetospeed(&m_options, B2400);
        break;

    case BaudRate::k4800:
        cfsetispeed(&m_options, B4800);
        cfsetospeed(&m_options, B4800);
        break;

    case BaudRate::k9600:
        cfsetispeed(&m_options, B9600);
        cfsetospeed(&m_options, B9600);
        break;

    case BaudRate::k19200:
        cfsetispeed(&m_options, B19200);
        cfsetospeed(&m_options, B19200);
        break;

    case BaudRate::k38400:
        cfsetispeed(&m_options, B38400);
        cfsetospeed(&m_options, B38400);
        break;

    case BaudRate::k57600:
        cfsetispeed(&m_options, B57600);
        cfsetospeed(&m_options, B57600);
        break;
    
    case BaudRate::k115200:
        cfsetispeed(&m_options, B115200);
        cfsetospeed(&m_options, B115200);
        break;
    } // switch (m_baudRate)

    // Set data bits
    // options_.c_cflag &= ~CSIZE; // WTF this doesn't support by kernel
    switch (m_dataBits) {    
    case DataBits::k5:
        m_options.c_cflag |= CS5;
        break;
    
    case DataBits::k6:
        m_options.c_cflag |= CS6;
        break;
    
    case DataBits::k7:        
        m_options.c_cflag |= CS7;
        break;
    
    case DataBits::k8:
        m_options.c_cflag |= CS8;
        break;
    } // switch (m_dataBits)  

    // Set parity
    switch (m_parity) {
    case Parity::kNone:
        m_options.c_cflag &= ~PARENB;
        break;

    case Parity::kEven:
        m_options.c_cflag |= PARENB;
        m_options.c_cflag &= ~PARODD;
        break;

    case Parity::kOdd:
        m_options.c_cflag |= PARENB;
        m_options.c_cflag |= PARODD;
        break;

    case Parity::kSpace:
        m_options.c_cflag &= ~PARENB;
        break;
    } // switch (m_parity)

    // Set stop bits
    switch(m_stopBits) {
    case StopBits::kOne:
        m_options.c_cflag &= ~CSTOPB;
        break;
    
    case StopBits::kTwo:
        m_options.c_cflag |= CSTOPB;
        break;
    } // switch (m_stopBits)

    // Apply options
    ret = tcsetattr(m_fd, TCSANOW, &m_options);
    if (ret == -1) {
        ::close(m_fd);
        m_fd = 0;
        return RetCode::kFailedToSetPortOptions;
    }

    // Send interrupt and push new instance with polling fd
    m_interruptMutex.lock();
    char code = '0';
    ::write(m_pipe[1], &code, sizeof(code));

    m_mutex.lock();
    m_pollFds.emplace_back(pollfd { m_fd, POLLIN, 0 });
    m_instances.emplace_back(this);

    m_mutex.unlock();
    m_interruptMutex.unlock();

    return RetCode::kSuccess;
}

void SerialPort::close() {
    if (m_fd == 0) {
        return;
    }

    // Send interrupt and remove instance with polling fd
    m_interruptMutex.lock();
    char code = '0';
    ::write(m_pipe[1], &code, sizeof(code));

    m_mutex.lock();
    for (size_t i = 1; i < m_pollFds.size(); ++i) {
        if (m_pollFds[i].fd != m_fd) {
            continue;
        }
        m_pollFds.erase(m_pollFds.begin() + i);
        m_instances.erase(m_instances.begin() + i);
        break;
    }

    ::close(m_fd);
    m_fd = 0;

    m_mutex.unlock();
    m_interruptMutex.unlock();
}

SerialPort::RetCode SerialPort::write(const void *data, size_t length) {
    ssize_t bytesWritten = ::write(m_fd, data, length);

    if (bytesWritten == -1) {
        switch (errno) {
        case EBADF:
            return RetCode::kPortNotOpened;
        default:
            return RetCode::kUndefinedError;
        }
    }

    if (static_cast<size_t>(bytesWritten) < length) {
        return RetCode::kNotAllWritten;
    }

    return RetCode::kSuccess;
}

/******************************************************************************
 * Methods (Protected)
 *****************************************************************************/

void SerialPort::polling() {
    char interrupt;
    int bufferSize = 1024;
    int ret = 0;
    size_t i = 0;
    ssize_t bytesRead = 0;
    std::shared_ptr<char[]> buffer(new char[bufferSize]);

    while (m_threadRunning) {
        m_mutex.lock();
        ret = poll(m_pollFds.data(), m_pollFds.size(), -1);
        m_mutex.unlock();

        if (ret < 1) {
            continue;
        }
        
        // Check "soft" interrupt
        if (m_pollFds[0].revents & POLLIN) {
            m_interruptMutex.lock();
            m_pollFds[0].revents = 0;
            read(m_pipe[0], &interrupt, sizeof(interrupt));
            m_interruptMutex.unlock();
            continue;
        }

        m_mutex.lock();
        for (i = 1; i < m_pollFds.size(); ++i) {
            if (!m_pollFds[i].revents & POLLIN) {
                continue;
            }
            
            m_pollFds[i].revents = 0;

            bytesRead = read(m_pollFds[i].fd, buffer.get(), bufferSize);

            if (bytesRead <= 0) {
                // Send error
                switch (bytesRead) {
                case 0:
                    m_instances[i]->onError(RetCode::kDeviceRemovedDuringOperation);
                    break;

                case -1:
                    m_instances[i]->onError(RetCode::kReadError);
                    break;
                }

                // Close serial port
                ::close(m_instances[i]->m_fd);
                m_instances[i]->m_fd = 0;
                
                m_pollFds.erase(m_pollFds.begin() + i);
                m_instances.erase(m_instances.begin() + i);
                --i;
                continue;
            }

            // Send data and allocate new buffer
            m_instances[i]->onData(buffer, bytesRead);
            buffer = std::shared_ptr<char[]>(new char[bufferSize]);
        }
        m_mutex.unlock();
    }
}

} // namespace UT