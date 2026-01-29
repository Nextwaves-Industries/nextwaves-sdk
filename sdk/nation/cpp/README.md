# Nation RFID SDK - C++

A cross-platform C++ SDK for NRN RFID readers.

## Requirements

- C++17 or later
- CMake 3.14+ (optional, for building)

## Files

- `nrn.hpp` - Header file with API definitions
- `nrn.cpp` - Implementation

## Quick Start

```cpp
#include "nrn.hpp"

int main() {
    nrn::NRNReader reader("/dev/ttyUSB0", 115200);
    
    if (!reader.open()) {
        return 1;
    }
    
    reader.connect_and_initialize();
    
    // Start inventory
    reader.start_inventory(0x01, [](const nrn::TagData& tag) {
        std::cout << "EPC: " << tag.epc;
        if (tag.rssi) {
            std::cout << ", RSSI: " << *tag.rssi << " dBm";
        }
        std::cout << std::endl;
    });
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    reader.stop_inventory();
    reader.close();
    
    return 0;
}
```

## RSSI Conversion

```cpp
// Raw RSSI to dBm
int rssi_dbm = nrn::calculate_rssi(raw_value);
// Formula: -100 + round((raw * 70) / 255)
```

## Building

```bash
g++ -std=c++17 -o example example.cpp nrn.cpp -lpthread
```

## Platform Support

- Linux: Uses termios for serial
- Windows: Uses Win32 API
- macOS: Uses termios (same as Linux)
