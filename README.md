# NextWaves Industries VMR64 RFID Reader SDK

This repository contains the Software Development Kit (SDK) for the NextWaves Industries VMR64 RFID Reader.

## Overview

The VMR64 is a high-performance UHF RFID reader designed for industrial applications. This SDK provides the necessary tools and libraries to integrate the VMR64 reader into your applications.

## Contents

- `/driver` - USB-to-UART bridge drivers (CP210x)
- `/sdk`
  - `/c#` - C# SDK
  - `c++` - C++ SDK
  - `python` - Python SDK class

## Requirements

- Windows 7/8/10/11 or linux, macOS
- .NET 8.0 or later (for C# applications)
- CP210x USB-to-UART bridge driver

## Getting Started

1. Install the CP210x USB-to-UART bridge driver from the `/driver` directory
2. Connect the VMR64 reader to your computer via USB
3. Build and run the sample applications in the SDK

## Sample Applications

### C# Windows Forms Application
Located in `/sdk/c#/APPForm/`, this sample demonstrates:
- Basic reader connectivity
- Tag reading operations
- Reader configuration
- Data processing

## Documentation

Detailed documentation for the SDK APIs and reader functionality will be provided in future updates.

## Support

For technical support and questions, please contact:
- Email: support@nextwaves.industries
- Website: www.nextwaves.industries

## License

Copyright © 2024 NextWaves Industries. All rights reserved.
