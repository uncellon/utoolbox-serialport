#ifndef UT_SERIAL_PORT
#define UT_SERIAL_PORT

#include <mutex>
#include <string>
#include <sys/poll.h>
#include <termios.h>
#include <thread>
#include <ut/core/event.h>
#include <vector>

namespace UT {

class SerialPort {
public:
    enum class BaudRate;
    enum class DataBits;
    enum class Parity;
    enum class StopBits;
    enum class RetCode;

    /***************************************************************************
     * Constructors / Destructors
     **************************************************************************/

    SerialPort();
    ~SerialPort();

    /***************************************************************************
     * Public methods
     **************************************************************************/

    /**
     * @brief Try to open serial port device
     * 
     * @param port path to the device file, e.g. "/dev/ttyS0", "/dev/ttyUSB0" etc
     * @return RetCode 
     */
    RetCode open(const std::string &port);

    void close();

    /**
     * @brief Write data to the serial port output buffer
     * 
     * This method is non-blocking and return code immediately. Serial port
     * driver is responsible for buffering and data transfer.
     * 
     * @param data data to be written
     * @param length length of data to be written
     * @return RetCode returns the following codes
     *     kSuccess - data written successfully, 
     *     kPortNotOpened - method open(...) not called, 
     *     kNotAllWritten - not all data was sended, may be returned when the 
     * driver buffer is full, 
     *     kUndefinedError - undefined error.
     */
    RetCode write(const void *data, size_t length);

    /**
     * @brief Write data to the serial port output buffer
     * 
     * This method is non-blocking and return code immediately. Serial port
     * driver is responsible for buffering and data transfer.
     * 
     * @param data data to be written
     * @param length length of data to be written
     * @return RetCode returns the following codes
     *     SUCCESS - data written successfully, 
     *     PORT_NOT_OPENED - method open(...) not called, 
     *     NOT_ALL_WRITTEN - not all data was sended, may be returned when the 
     * driver buffer is full,
     *     UNDEFINED_ERROR - undefined error.
     */
    RetCode write(const char *data, size_t length);

    /***************************************************************************
     * Events
     **************************************************************************/

    Event<std::shared_ptr<char[]>, ssize_t> onData;
    Event<RetCode> onError;

    /***************************************************************************
     * Accessors / Mutators
     **************************************************************************/

    BaudRate baudRate() const;
    void setBaudRate(BaudRate baudRate);

    DataBits dataBits() const;
    void setDataBits(DataBits dataBits);

    Parity parity() const;
    void setParity(Parity parity);

    StopBits stopBits() const;
    void setStopBits(StopBits stopBits);

protected:
    /**************************************************************************
     * Methods (Protected)
     *************************************************************************/

    static void polling();

    /**************************************************************************
     * Members
     *************************************************************************/

    static bool m_threadRunning;            // thread quit confition
    static int m_pipe[2];
    static std::mutex m_interruptMutex;
    static std::mutex m_mutex;               // vectors protection mutex
    static std::mutex m_threadInstanceMutex;
    static std::thread *m_pollingThread;    // singleton thread to polling ports
    static std::vector<SerialPort *> m_instances;    // port instances (non-static objects)
    static std::vector<pollfd> m_pollFds;   // polling file descriptors

    BaudRate m_baudRate;
    DataBits m_dataBits;
    Parity m_parity;
    StopBits m_stopBits;
    int m_fd = 0;
    termios m_options;
};

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
};

enum class SerialPort::DataBits {
    k5 = 5,
    k6 = 6,
    k7 = 7,
    k8 = 8
};

enum class SerialPort::Parity {
    kNone,
    kEven,
    kOdd,
    kSpace
};

enum class SerialPort::StopBits {
    kOne,
    kTwo
};

/** Return codes enumeration */
enum class SerialPort::RetCode {
    kSuccess = 0,                       /**< Successful operation */
    kAlreadyOpened = 1,
    kDeviceNotConnected = 2,
    kNotAllWritten = 3,
    kPortNotOpened = 4,
    kDeviceRemovedDuringOperation = 5,
    kReadError = 6,
    kBufferFlushError = 7,
    kSwitchToNonBlockingModeError = 8,
    kFailedToGetPortOptions = 9,
    kFailedToSetPortOptions = 10,
    kPipeFailed,
    kUndefinedError
};

/*******************************************************************************
 * Inline
 ******************************************************************************/

inline SerialPort::RetCode SerialPort::write(const char *data, size_t length) { 
    return write(static_cast<const void *>(data), length); 
}

inline SerialPort::BaudRate SerialPort::baudRate() const { return m_baudRate; }
inline void SerialPort::setBaudRate(SerialPort::BaudRate baudRate) { m_baudRate = baudRate; }

inline SerialPort::DataBits SerialPort::dataBits() const { return m_dataBits; }
inline void SerialPort::setDataBits(SerialPort::DataBits dataBits) { m_dataBits = dataBits; }

inline SerialPort::Parity SerialPort::parity() const { return m_parity; }
inline void SerialPort::setParity(SerialPort::Parity parity) { m_parity = parity; }

inline SerialPort::StopBits SerialPort::stopBits() const { return m_stopBits; }
inline void SerialPort::setStopBits(SerialPort::StopBits stopBits) { m_stopBits = stopBits; }

} // namespace UT

#endif // UT_SERIAL_PORT