#include <ut/serialport/serialport.h>

#include <cmath>
#include <string>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <termios.h>

using namespace UT;

std::condition_variable cv;

int main(int argc, char* argv[]) {
    EventLoop mainLoop;

    SerialPort sp;
    sp.setBaudRate(SerialPort::BaudRate::k115200);
    sp.setDataBits(SerialPort::DataBits::k6);
    sp.setParity(SerialPort::Parity::kNone);
    sp.setStopBits(SerialPort::StopBits::kOne);

    auto ret = sp.open("/dev/ttyUSB0");
    if (ret != SerialPort::RetCode::kSuccess) {
        switch (ret) {
        case SerialPort::RetCode::kDeviceNotConnected:
            std::cout << "Device not connected!\n";
            return EXIT_FAILURE;
        default:
            std::cout << "Serial port open failed: unknown error\n";
            return EXIT_FAILURE;
        }
    } else {
        std::cout << "Serial port ttyUSB0 opened\n";
    }

    sp.onData.addEventHandler(&mainLoop, [] (std::shared_ptr<char[]> data, size_t length) {
        auto cdata = data.get();
        std::cout << "Bytes read: " << std::to_string(length) << std::endl << "data: ";
        for (size_t i = 0; i < length; ++i) {
            std::cout << std::setfill('0') << std::setw(2) << std::hex << int(cdata[i]) << " ";
        }
        std::cout << std::endl;
        cv.notify_one();
    });

    sp.onError.addEventHandler(&mainLoop, [] (SerialPort::RetCode code) {
        switch (code) {
        case SerialPort::RetCode::kDeviceRemovedDuringOperation:
            std::cout << "Device removed during operation\n";
            break;
        
        default:
            break;
        }
    });

    auto weightRequest = "\x47\x44\x08\x00\x53\x45\x52\x3F\x52\x23\x40\x02\x48\xC4"; // length 14
    auto taringRequest = "\x47\x44\x0C\x00\x53\x45\x52\x3F\x57\x23\x20\x01\x00\x00\x00\x00\xDE\x06"; // length 18

    // send taring request
    sp.write(taringRequest, 18);
    std::cout << "Taring request sended\n";
    // taring response 47 44 08 00 A4 A2 5E 54 57 45 20 01 3C BF

    // sleep for 1 second
    sleep(1);

    // send weight request
    auto result = sp.write(weightRequest, 14);
    if (result != SerialPort::RetCode::kSuccess) {
        std::cout << "Write error occured\n";
    }
    std::cout << "Weight request sended\n";
    // weight response 47 44 0C 00 A4 A2 5E 54 52 45 40 02 00 00 00 00 B5 EF
    //                 47 44 0C 00 A4 A2 5E 54 52 45 40 02 A2 FF FF FF A7 D7
    //                 47 44 0C 00 A4 A2 5E 54 52 45 40 02 8A 03 00 00 6F F7

    std::mutex mutex;
    std::unique_lock lock(mutex);
    cv.wait(lock);

    return 0;
}