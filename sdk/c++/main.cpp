#include "inc/serial/serial.h"
#include "inc/reader/ReaderMethod.h"
#include "inc/reader/ReaderProcessor.h"
#include "inc/reader/MessageTran.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <iostream>
#include "thread"
int main() {
    

    // if (my_serial.isOpen()) {
    //     std::cout << "Port opened successfully" << std::endl;
    // } else {
    //     std::cout << "Port failed to open" << std::endl;
    //     return 1; 
    // }

    // my_serial.flushOutput();

    // std::string test_string = "Hello, Arduino!\n";
    // while (true) {
    //     size_t bytes_written = my_serial.write(test_string);
    //     std::cout << "Bytes sent: " << bytes_written << std::endl;

    //     std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    //     my_serial.flushInput();
    //     std::string response = my_serial.read(6);
    //     std::cout << "Arduino: " << response << std::endl;

    //     if (response == "exit") {
    //         std::cout << "Ending communication..." << std::endl;
    //         break;
    //     }
    // }

    return 0;
}