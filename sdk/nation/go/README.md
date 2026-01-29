# Nation RFID SDK - Go

A Go SDK for NRN RFID readers using the `go.bug.st/serial` package.

## Installation

```bash
go get github.com/nextwaves/nrn-sdk
```

Or add to your `go.mod`:

```go
require github.com/nextwaves/nrn-sdk v1.0.0
```

## Quick Start

```go
package main

import (
    "fmt"
    "time"
    
    nrn "github.com/nextwaves/nrn-sdk"
)

func main() {
    reader, err := nrn.NewNRNReader("/dev/ttyUSB0", 115200)
    if err != nil {
        panic(err)
    }
    defer reader.Close()
    
    if err := reader.ConnectAndInitialize(); err != nil {
        panic(err)
    }
    
    // Start inventory on antenna 1
    err = reader.StartInventory(0x01, func(tag nrn.TagData) {
        fmt.Printf("EPC: %s", tag.EPC)
        if tag.RSSI != nil {
            fmt.Printf(", RSSI: %d dBm", *tag.RSSI)
        }
        fmt.Println()
    })
    if err != nil {
        panic(err)
    }
    
    time.Sleep(5 * time.Second)
    reader.StopInventory()
}
```

## Utility Functions

```go
import nrn "github.com/nextwaves/nrn-sdk"

// RSSI conversion: -100 + round((raw * 70) / 255)
rssiDBm := nrn.CalculateRSSI(128)  // -65 dBm

// Frequency conversion: 920.0 + idx * 0.5
freqMHz := nrn.CalculateFrequency(10)  // 925.0 MHz

// Build antenna mask from list
mask := nrn.BuildAntennaMask([]int{1, 2})  // 0x00000003
```

## Features

- Cross-platform serial communication
- All NATION protocol MID commands
- Tag parsing with optional TID, phase, frequency
- Thread-safe inventory operations
