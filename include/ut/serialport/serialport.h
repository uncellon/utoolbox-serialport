/******************************************************************************
 * 
 * Copyright (C) 2023 Dmitry Plastinin
 * Contact: uncellon@yandex.ru, uncellon@gmail.com, uncellon@mail.ru
 * 
 * This file is part of the UToolbox SerialPort library.
 * 
 * UToolbox SerialPort is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU Lesser General Public License as pubblished by
 * the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * UToolbox SerialPort is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser Public License for 
 * more details
 * 
 * You should have received a copy of the GNU Lesset General Public License
 * along with UToolbox SerialPort. If not, see <https://www.gnu.org/licenses/>.
 * 
 *****************************************************************************/

#ifndef UT_SERIAL_PORT
#define UT_SERIAL_PORT

#include <sys/poll.h>
#include <termios.h>
#include <ut/core/event.h>

namespace UT {

class SerialPort {
public:
    enum class BaudRate;
    enum class DataBits;
    enum class Parity;
    enum class StopBits;
    enum class Opcode;

    /**************************************************************************
     * Constructors / Destructors
     *************************************************************************/

    SerialPort();
    SerialPort(const SerialPort& other) = delete;
    SerialPort(SerialPort&& other) = delete;
    ~SerialPort();

    /**************************************************************************
     * Methods
     *************************************************************************/

    /**
     * @brief Try to open serial port device
     * 
     * @param port path to the device file, e.g. "/dev/ttyS0", "/dev/ttyUSB0", etc.
     * @return SerialPort::Opcode 
     *     - kSuccess - device successfully opened
     *     - kAlreadyOpened - device already open by this instance
     *     - kDeviceDoesNotExist - device not connected
     *     - kBufferFlushError - ioctl(..., TCFLSH, TCIOFLUSH) failed
     *     - kFailedToGetPortOptions - tcgetattr(...) failed
     *     - kFailedToSetPortOptions - tcsetattr(...) failed
     *     - kUndefinedError - undefined error
     */
    Opcode open(const std::string& port);

    void close();

    /**
     * @brief Write data to the serial port output buffer
     * 
     * This method is non-blocking and return code immediately. Serial port
     * driver is responsible for buffering and data transfer.
     * 
     * @param data data to be written
     * @param length length of data to be written
     * @return Opcode returns the following codes
     *     - kSuccess - data written successfully
     *     - kPortNotOpened - method open(...) not called
     *     - kNotAllWritten - not all data was sended, may be returned when the 
     * driver buffer is full
     *     - kUndefinedError - undefined error
     */
    Opcode write(const void* data, size_t length);

    /**
     * @brief Helper for write(const void* data, size_t length)
     */
    Opcode write(const char* data, size_t length);

    /**************************************************************************
     * Events
     *************************************************************************/

    Event<std::shared_ptr<void>, size_t> onData;
    Event<Opcode> onError;

    /**************************************************************************
     * Accessors / Mutators
     *************************************************************************/

    BaudRate getBaudRate() const;
    void setBaudRate(BaudRate baudRate);

    DataBits getDataBits() const;
    void setDataBits(DataBits dataBits);

    Parity getParity() const;
    void setParity(Parity parity);

    StopBits getStopBits() const;
    void setStopBits(StopBits stopBits);

protected:
    /**************************************************************************
     * Methods (Protected)
     *************************************************************************/

    static void polling();

    /**************************************************************************
     * Members
     *************************************************************************/

    static bool mRunning;
    static int mPipe[2];
    static std::mutex mInterruptMutex;
    static std::mutex mMutex;
    static std::mutex mCdtorsMutex;
    static std::thread* mSerialThread;
    static std::vector<SerialPort*> mInstances;
    static std::vector<pollfd> mPfds;

    BaudRate mBaudRate;
    DataBits mDataBits;
    Parity mParity;
    StopBits mStopBits;
    int mFd = -1;
    termios mOptions;

}; // class SerialPort

enum class SerialPort::BaudRate {
    k0 = 0,
    k50 = 50,
    k75 = 75,
    k110 = 110,
    k134 = 134,
    k150 = 150,
    k200 = 200,
    k300 = 300,
    k600 = 600,
    k1200 = 1200,
    k1800 = 1800,
    k2400 = 2400,
    k4800 = 4800,
    k9600 = 9600,
    k19200 = 19200,
    k38400 = 38400,
    k57600 = 57600,
    k115200 = 115200
}; // enum class SerialPort::BaudRate

enum class SerialPort::DataBits {
    k5 = 5,
    k6 = 6,
    k7 = 7,
    k8 = 8
}; // enum class SerialPort::DataBits

enum class SerialPort::Parity {
    kNone,
    kEven,
    kOdd,
    kSpace
}; // enum class SerialPort::Parity

enum class SerialPort::StopBits {
    kOne,
    kTwo
}; // enum class SerialPort::StopBits

/** Return codes enumeration */
enum class SerialPort::Opcode {
    kSuccess,                       /**< Successful operation */
    kAlreadyOpened,
    kDeviceDoesNotExist,
    kNotAllWritten,
    kPortNotOpened,
    kDeviceRemovedDuringOperation,
    kReadError,
    kBufferFlushError,
    kFailedToGetPortOptions,
    kFailedToSetPortOptions,
    kUndefinedError
}; // enum class SerialPort::Opcode

/*******************************************************************************
 * Inline definition
 ******************************************************************************/

inline SerialPort::Opcode SerialPort::write(const char *data, size_t length) { 
    return write(static_cast<const void *>(data), length); 
}

inline SerialPort::BaudRate SerialPort::getBaudRate() const { return mBaudRate; }
inline void SerialPort::setBaudRate(SerialPort::BaudRate baudRate) { mBaudRate = baudRate; }

inline SerialPort::DataBits SerialPort::getDataBits() const { return mDataBits; }
inline void SerialPort::setDataBits(SerialPort::DataBits dataBits) { mDataBits = dataBits; }

inline SerialPort::Parity SerialPort::getParity() const { return mParity; }
inline void SerialPort::setParity(SerialPort::Parity parity) { mParity = parity; }

inline SerialPort::StopBits SerialPort::getStopBits() const { return mStopBits; }
inline void SerialPort::setStopBits(SerialPort::StopBits stopBits) { mStopBits = stopBits; }

} // namespace UT

#endif // UT_SERIAL_PORT