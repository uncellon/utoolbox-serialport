#include <iostream>
#include <unistd.h>
#include <ut/serialport/serialport.h>

using namespace UT;

int main(int argc, char* argv[]) {
    EventLoop mainLoop;

    SerialPort sp;
    sp.setBaudRate(SerialPort::BaudRate::k115200);
    sp.setDataBits(SerialPort::DataBits::k8);
    sp.setParity(SerialPort::Parity::kNone);
    sp.setStopBits(SerialPort::StopBits::kOne);

    sp.onData.addEventHandler(
        EventLoop::getMainInstance(), 
        [] (std::shared_ptr<void> data, size_t length) {
            std::cout << "Received: " << static_cast<char*>(data.get()) << std::endl;
        }
    );

    sp.onError.addEventHandler(
        EventLoop::getMainInstance(), 
        [] (SerialPort::Opcode code) {
            switch (code) {
            case SerialPort::Opcode::kDeviceRemovedDuringOperation:
                std::cout << "Device removed during operation\n";
                break;        
            default:
                break;
            }
        }
    );

    auto ret = sp.open("/dev/ttyUSB0");
    if (ret != SerialPort::Opcode::kSuccess) {
        switch (ret) {
        case SerialPort::Opcode::kDeviceDoesNotExist:
            std::cout << "Device not connected!\n";
            break;
        default:
            std::cout << "Serial port open failed: unknown error\n";
            break;
        }
        return EXIT_FAILURE;
    }

    // Arduino boot delay
    sleep(2);

    for (int i = 0; i < 10; ++i) {
        ret = sp.write("WND", 3);
        if (ret != SerialPort::Opcode::kSuccess) {
            std::cout << "Error occured\n";
            return EXIT_FAILURE;
        }
        sleep(1);
    }

    sleep(2);

    return EXIT_SUCCESS;
}