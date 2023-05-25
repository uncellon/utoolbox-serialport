# Uncellon's Toolbox SerialPort

![UToolbox Logo](logo.png)

- [Description](#description)
- [Prerequisites](#prerequisites)
- [Example](#example)
- [License](#license)

## Description

Library for communicating with serial devices.

## Prerequisites

- C++17 or higher
- CMake >= 3.16
- [UToolbox Core](https://github.com/uncellon/utoolbox-core) >= 0.0.10

## Example

```cpp
#include <iostream>
#include <unistd.h>
#include <ut/serialport/serialport.h>

using namespace UT;

int main(int argc, char* argv[]) {
    // Configure serial port
    SerialPort sp;
    sp.setBaudRate(SerialPort::BaudRate::k115200);
    sp.setDataBits(SerialPort::DataBits::k8);
    sp.setParity(SerialPort::Parity::kNone);
    sp.setStopBits(SerialPort::StopBits::kOne);

    // Add data handler
    sp.onData.addEventHandler(
        EventLoop::getMainInstance(), 
        [] (std::shared_ptr<void> data, size_t length) {
            std::cout << "Received: " << static_cast<char*>(data.get()) << std::endl;
        }
    );

    // Add error handler (reading errors only)
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

    // Open device
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

    // Write
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
```

## License

<img align="right" src="https://www.gnu.org/graphics/lgplv3-with-text-154x68.png">

The library is licensed under [GNU Lesser General Public License 3.0](https://www.gnu.org/licenses/lgpl-3.0.txt):

Copyright © 2023 Dmitry Plastinin

UToolbox Timers is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as pubblished by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

UToolbox Timers is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser Public License for more details