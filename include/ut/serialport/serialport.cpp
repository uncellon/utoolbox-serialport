#include "serialport.h"

#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace UT {

bool SerialPort::mRunning = false;
int SerialPort::mPipe[2] = { 0, 0 };
std::mutex SerialPort::mInterruptMutex;
std::mutex SerialPort::mMutex;
std::mutex SerialPort::mCdtorsMutex;
std::thread *SerialPort::mSerialThread = nullptr;
std::vector<SerialPort *> SerialPort::mInstances;
std::vector<pollfd> SerialPort::mPfds;

SerialPort::SerialPort()
: mBaudRate(BaudRate::k115200),
  mDataBits(DataBits::k8),
  mParity(Parity::kNone),
  mStopBits(StopBits::kOne) {
    memset(&mOptions, 0, sizeof(mOptions));

    std::unique_lock lock(mCdtorsMutex);

    if (mSerialThread) {
        return;
    }

    // Create pipes
    auto ret = pipe2(mPipe, O_NONBLOCK);
    if (ret == -1) {
        throw std::runtime_error("pipe(...) failed");
    }

    // Insert null-instance and pipe polling fd
    mPfds.clear();
    mInstances.clear();

    mPfds.emplace_back(pollfd { mPipe[0], POLLIN, 0 });
    mInstances.emplace_back(nullptr);

    // Start polling thread
    mRunning = true;
    mSerialThread = new std::thread(polling);
}

SerialPort::~SerialPort() {
    close();

    std::unique_lock lock(mCdtorsMutex);

    if (mPfds.size()) {
        return;
    }

    // Send interrupt and delete thread
    mRunning = false;
    char code = '0';
    ::write(mPipe[1], &code, sizeof(code));
    if (mSerialThread) {
        mSerialThread->join();
    }
    delete mSerialThread;
    mSerialThread = nullptr;

    // Close pipes
    ::close(mPipe[0]);
    ::close(mPipe[1]);
}

SerialPort::Opcode SerialPort::open(const std::string& port) {
    if (mFd != -1) {
        return Opcode::kAlreadyOpen;
    }

    mFd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (mFd == -1) {
        switch (errno) {
        case ENOENT:
            return Opcode::kDeviceNotFound;
        default:
            return Opcode::kSyscallError;
        }
    }

    // Clear I/O buffers
    auto ret = ioctl(mFd, TCFLSH, TCIOFLUSH);
    if (ret == -1) {
        ::close(mFd);
        mFd = -1;
        return Opcode::kSyscallError;
    }

    // Get current serial port options
    ret = tcgetattr(mFd, &mOptions);
    if (ret == -1) {
        ::close(mFd);
        mFd = -1;
        return Opcode::kSyscallError;
    }

    // Set local mode and enable receiver
    mOptions.c_cflag |= (CLOCAL | CREAD);

    // To raw mode
    cfmakeraw(&mOptions);

    // Set speed (baud)
    switch (mBaudRate) {
    case BaudRate::k0:
        cfsetispeed(&mOptions, B0);
        cfsetospeed(&mOptions, B0);
        break;        
    case BaudRate::k50:
        cfsetispeed(&mOptions, B50);
        cfsetospeed(&mOptions, B50);
        break;
    case BaudRate::k75:
        cfsetispeed(&mOptions, B75);
        cfsetospeed(&mOptions, B75);
        break;
    case BaudRate::k110:
        cfsetispeed(&mOptions, B110);
        cfsetospeed(&mOptions, B110);
        break;
    case BaudRate::k134:
        cfsetispeed(&mOptions, B134);
        cfsetospeed(&mOptions, B134);
        break;
    case BaudRate::k150:
        cfsetispeed(&mOptions, B150);
        cfsetospeed(&mOptions, B150);
        break;
    case BaudRate::k200:
        cfsetispeed(&mOptions, B200);
        cfsetospeed(&mOptions, B200);
        break;
    case BaudRate::k300:
        cfsetispeed(&mOptions, B300);
        cfsetospeed(&mOptions, B300);
        break;
    case BaudRate::k600:
        cfsetispeed(&mOptions, B600);
        cfsetospeed(&mOptions, B600);
        break;
    case BaudRate::k1200:
        cfsetispeed(&mOptions, B1200);
        cfsetospeed(&mOptions, B1200);
        break;
    case BaudRate::k1800:
        cfsetispeed(&mOptions, B1800);
        cfsetospeed(&mOptions, B1800);
        break;
    case BaudRate::k2400:
        cfsetispeed(&mOptions, B2400);
        cfsetospeed(&mOptions, B2400);
        break;
    case BaudRate::k4800:
        cfsetispeed(&mOptions, B4800);
        cfsetospeed(&mOptions, B4800);
        break;
    case BaudRate::k9600:
        cfsetispeed(&mOptions, B9600);
        cfsetospeed(&mOptions, B9600);
        break;
    case BaudRate::k19200:
        cfsetispeed(&mOptions, B19200);
        cfsetospeed(&mOptions, B19200);
        break;
    case BaudRate::k38400:
        cfsetispeed(&mOptions, B38400);
        cfsetospeed(&mOptions, B38400);
        break;
    case BaudRate::k57600:
        cfsetispeed(&mOptions, B57600);
        cfsetospeed(&mOptions, B57600);
        break;    
    case BaudRate::k115200:
        cfsetispeed(&mOptions, B115200);
        cfsetospeed(&mOptions, B115200);
        break;
    } // switch (m_baudRate)

    // Set data bits
    // options_.c_cflag &= ~CSIZE; // WTF this doesn't support by kernel
    switch (mDataBits) {    
    case DataBits::k5:
        mOptions.c_cflag |= CS5;
        break;    
    case DataBits::k6:
        mOptions.c_cflag |= CS6;
        break;    
    case DataBits::k7:        
        mOptions.c_cflag |= CS7;
        break;    
    case DataBits::k8:
        mOptions.c_cflag |= CS8;
        break;
    } // switch (m_dataBits)  

    // Set parity
    switch (mParity) {
    case Parity::kNone:
        mOptions.c_cflag &= ~PARENB;
        break;
    case Parity::kEven:
        mOptions.c_cflag |= PARENB;
        mOptions.c_cflag &= ~PARODD;
        break;
    case Parity::kOdd:
        mOptions.c_cflag |= PARENB;
        mOptions.c_cflag |= PARODD;
        break;
    case Parity::kSpace:
        mOptions.c_cflag &= ~PARENB;
        break;
    } // switch (m_parity)

    // Set stop bits
    switch(mStopBits) {
    case StopBits::kOne:
        mOptions.c_cflag &= ~CSTOPB;
        break;    
    case StopBits::kTwo:
        mOptions.c_cflag |= CSTOPB;
        break;
    } // switch (m_stopBits)

    // Apply options
    ret = tcsetattr(mFd, TCSANOW, &mOptions);
    if (ret == -1) {
        ::close(mFd);
        mFd = -1;
        return Opcode::kSyscallError;
    }

    // Send interrupt and push new instance with polling fd
    mInterruptMutex.lock();
    char code = '0';
    ::write(mPipe[1], &code, sizeof(code));

    mMutex.lock();
    mPfds.emplace_back(pollfd { mFd, POLLIN, 0 });
    mInstances.emplace_back(this);

    mMutex.unlock();
    mInterruptMutex.unlock();

    return Opcode::kSuccess;
}

void SerialPort::close() {
    if (mFd == -1) {
        return;
    }

    // Send interrupt and remove instance with polling fd
    mInterruptMutex.lock();
    char code = '0';
    ::write(mPipe[1], &code, sizeof(code));

    mMutex.lock();
    for (size_t i = 1; i < mPfds.size(); ++i) {
        if (mPfds[i].fd != mFd) {
            continue;
        }
        mPfds.erase(mPfds.begin() + i);
        mInstances.erase(mInstances.begin() + i);
        break;
    }

    ::close(mFd);
    mFd = -1;

    mMutex.unlock();
    mInterruptMutex.unlock();
}

SerialPort::Opcode SerialPort::write(const void* data, size_t length) {
    ssize_t bytesWritten = ::write(mFd, data, length);

    if (bytesWritten == -1) {
        switch (errno) {
        case EBADF:
            return Opcode::kDeviceNotOpen;
        default:
            return Opcode::kSyscallError;
        }
    }

    if (static_cast<size_t>(bytesWritten) < length) {
        return Opcode::kNotAllWritten;
    }

    return Opcode::kSuccess;
}

/******************************************************************************
 * Methods (Protected)
 *****************************************************************************/

void SerialPort::polling() {
    char interrupt;
    int kBufferSize = 1024;
    int ret = 0;
    size_t i = 0;

    while (mRunning) {
        mMutex.lock();
        ret = poll(mPfds.data(), mPfds.size(), -1);
        mMutex.unlock();

        if (ret < 1) {
            continue;
        }
        
        // Check "soft" interrupt
        if (mPfds[0].revents & POLLIN) {
            mInterruptMutex.lock();
            mPfds[0].revents = 0;            
            do {
                ret = read(mPipe[0], &interrupt, sizeof(interrupt));
            } while (ret > 0);
            
            mInterruptMutex.unlock();
            continue;
        }

        mMutex.lock();
        for (i = 1; i < mPfds.size(); ++i) {
            if (!mPfds[i].revents & POLLIN) {
                continue;
            }
            
            mPfds[i].revents = 0;

            while (true) {
                void* data = malloc(kBufferSize);

                ret = read(mPfds[i].fd, data, kBufferSize);
                if (ret > 0) {
                    mInstances[i]->onData(std::shared_ptr<void>(data, [] (void* data) { free(data); }), ret);
                    
                    if (ret == kBufferSize) {
                        continue;
                    }
                } else {
                    free(data);

                    if (ret == -1 && errno == EAGAIN) {
                        break;
                    }

                    if (ret == 0) {
                        mInstances[i]->onError(Opcode::kDeviceRemoved);
                    } else if (ret == -1) {
                        mInstances[i]->onError(Opcode::kReadError);
                    }

                    // Close serial port
                    ::close(mInstances[i]->mFd);
                    mInstances[i]->mFd = -1;
                    
                    mPfds.erase(mPfds.begin() + i);
                    mInstances.erase(mInstances.begin() + i);
                    --i;
                }

                break;
            }
        }
        mMutex.unlock();
    }
}

} // namespace UT