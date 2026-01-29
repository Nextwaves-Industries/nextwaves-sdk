# Nation RFID SDK - Rust

A Rust SDK for NRN RFID readers using the `serialport` crate.

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
nrn-sdk = { path = "../sdk/nation/rust" }
```

## Quick Start

```rust
use nrn_sdk::{NRNReader, TagData, build_antenna_mask};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut reader = NRNReader::new("/dev/ttyUSB0", 115200)?;
    reader.connect_and_initialize()?;
    
    // Start inventory on antenna 1
    reader.start_inventory(0x01, |tag: TagData| {
        println!("EPC: {}", tag.epc);
        if let Some(rssi) = tag.rssi {
            println!("RSSI: {} dBm", rssi);
        }
    })?;
    
    std::thread::sleep(std::time::Duration::from_secs(5));
    reader.stop_inventory()?;
    
    Ok(())
}
```

## Utility Functions

```rust
use nrn_sdk::{calculate_rssi, calculate_frequency, build_antenna_mask};

// RSSI conversion: -100 + round((raw * 70) / 255)
let rssi_dbm = calculate_rssi(128);  // -65 dBm

// Frequency conversion: 920.0 + idx * 0.5
let freq_mhz = calculate_frequency(10);  // 925.0 MHz

// Build antenna mask
let mask = build_antenna_mask(&[1, 2]);  // 0x00000003
```

## Running Tests

```bash
cargo test
```

## Features

- Cross-platform serial communication
- All NATION protocol MID commands
- Tag parsing with optional TID, phase, frequency
- Error handling with `thiserror`
