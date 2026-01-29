# Nextwaves Industries RFID Reader SDK

Official SDK for integrating Nextwaves Industries RFID Readers into your applications.

> **🚀 Quick Start:** Visit [app.nextwaves.com](https://app.nextwaves.com) to use the reader directly in your browser — no installation required!

## Overview

The Nextwaves Reader (NWR) is a high-performance UHF RFID reader designed for industrial applications. This SDK provides libraries and tools across multiple programming languages to integrate the reader into your projects.

## SDK Languages

| Language | Path | Description |
|----------|------|-------------|
| **C#** | `/sdk/nation/csharp/` | .NET SDK with Windows Forms sample |
| **C++** | `/sdk/nation/cpp/` | Header-only C++ implementation |
| **Python** | `/sdk/nation/python/` | Python SDK class |
| **Rust** | `/sdk/nation/rust/` | Rust crate implementation |
| **Go** | `/sdk/nation/go/` | Go module implementation |
| **TypeScript** | `/sdk/nation/webserial/` | Web Serial API for browser use |

## Drivers

- `/driver` - USB-to-UART bridge drivers (CP210x)

## Requirements

| Platform | Requirements |
|----------|--------------|
| **Windows** | Windows 7/8/10/11, CP210x driver |
| **Linux** | Most distributions (driver usually built-in) |
| **macOS** | macOS 10.9+, CP210x driver |
| **Web** | Chrome/Edge with Web Serial API support |

## Getting Started

1. **Install Driver** — Install the CP210x USB-to-UART bridge driver from `/driver` (if needed)
2. **Connect Reader** — Connect the reader to your computer via USB
3. **Choose SDK** — Pick the SDK for your preferred language
4. **Run Examples** — Build and run the sample applications

## Documentation

API documentation and protocol specifications are available in the `/docs` directory.

## Support

- **Email:** tech@nextwaves.industries
- **Website:** [nextwaves.com](https://nextwaves.com)
- **Web App:** [app.nextwaves.com](https://app.nextwaves.com)

## License

Copyright © 2024-2026 Nextwaves Industries. All rights reserved.
