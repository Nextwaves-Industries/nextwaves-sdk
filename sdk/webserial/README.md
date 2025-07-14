# NextWaves VMR64 RFID Reader - WebSerial Interface

This is a JavaScript/HTML implementation of the NextWaves VMR64 RFID reader communication protocol using the WebSerial API. It provides a web-based interface for reading RFID tags through a browser.

## Features

- **WebSerial Communication**: Uses the modern WebSerial API for direct serial communication
- **Real-time Tag Reading**: Continuously reads and displays RFID tags
- **Power Control**: Adjustable power settings for all 4 antennas (0-33 dBm)
- **Inventory Control**: Start/stop inventory operations
- **Reader Information**: Get device information
- **Modern UI**: Beautiful, responsive interface with real-time updates
- **Communication Log**: Detailed logging of all serial communication

## Requirements

- **Modern Browser**: Chrome 89+, Edge 89+, or Opera 76+ (WebSerial API support)
- **HTTPS or localhost**: WebSerial API requires a secure context
- **USB Serial Device**: NextWaves VMR64 RFID reader connected via USB

## Browser Compatibility

| Browser | Version | Status |
|---------|---------|--------|
| Chrome | 89+ | ✅ Supported |
| Edge | 89+ | ✅ Supported |
| Opera | 76+ | ✅ Supported |
| Firefox | Any | ❌ Not Supported |
| Safari | Any | ❌ Not Supported |

## Installation & Usage

1. **Download Files**: Ensure you have both `index.html` and `vmr64.js` in the same directory

2. **Serve Files**: You need to serve these files from a web server (not just open the HTML file directly)
   - **Option 1**: Use a local web server
     ```bash
     # Using Python 3
     python -m http.server 8000
     
     # Using Node.js (if you have http-server installed)
     npx http-server
     ```
   - **Option 2**: Use VS Code Live Server extension
   - **Option 3**: Deploy to a web server with HTTPS

3. **Access the Interface**: Open your browser and navigate to:
   - `http://localhost:8000` (if using Python server)
   - Or your deployed URL

4. **Connect to Reader**:
   - Click "Connect to Reader"
   - Select your USB serial device from the browser's port selection dialog
   - The interface will automatically connect and show "Connected" status

## Usage Instructions

### Connection
1. Click "Connect to Reader" button
2. Select your VMR64 device from the port selection dialog
3. Wait for connection confirmation

### Power Settings
- Adjust power levels for each antenna (0-33 dBm)
- Default values: Antenna 1-3: 30 dBm, Antenna 4: 10 dBm
- Click "Set Power" to apply settings

### Inventory Operations
- **Start Inventory**: Begins continuous tag reading
- **Stop Inventory**: Stops the inventory operation
- **Get Reader Info**: Retrieves device information

### Tag Display
- Detected tags are shown in real-time
- Each tag displays:
  - EPC (Electronic Product Code)
  - Antenna number
  - Frequency (MHz)
  - RSSI (signal strength)
  - Timestamp

### Communication Log
- All serial communication is logged
- Shows sent and received data in hexadecimal format
- Useful for debugging and monitoring

## Protocol Implementation

The JavaScript implementation follows the same protocol as the Python version:

### Message Structure
```
[0xA0][Length][ReaderID][Command][Data...][Checksum]
```

### Commands
- `0x01`: Get reader information
- `0x07`: Set power levels
- `0x8A`: Start inventory
- `0x8B`: Stop inventory

### Checksum Calculation
```javascript
checksum = (~sum + 1) & 0xff
```

## Troubleshooting

### Connection Issues
1. **Browser not supported**: Ensure you're using Chrome, Edge, or Opera
2. **HTTPS required**: Make sure you're accessing via HTTPS or localhost
3. **Device not detected**: Check USB connection and drivers
4. **Permission denied**: Allow port access when prompted

### No Tags Detected
1. **Check power settings**: Ensure antennas have sufficient power
2. **Verify tags**: Make sure RFID tags are within range
3. **Check inventory status**: Ensure inventory is started
4. **Review log**: Check communication log for errors

### Performance Issues
1. **Close other applications**: Free up system resources
2. **Check browser performance**: Monitor browser's task manager
3. **Reduce log verbosity**: The log can be resource-intensive

## Development

### File Structure
```
webserial/
├── index.html      # Main HTML interface
├── vmr64.js        # JavaScript implementation
└── README.md       # This file
```

### Key Classes
- **MessageTran**: Handles message creation and checksum calculation
- **NextWavesReader**: Main reader communication class
- **ReaderUI**: User interface controller

### Extending Functionality
To add new commands or features:
1. Add command constants in `NextWavesReader`
2. Implement message creation in `MessageTran`
3. Add UI controls in `ReaderUI`
4. Handle responses in `processFrame()`

## Security Notes

- WebSerial API requires user permission for port access
- All communication is local (no data sent to external servers)
- Serial port access is limited to the web page that requested it

## License

This implementation is based on the NextWaves VMR64 Python SDK and follows the same communication protocol.

## Support

For issues related to:
- **WebSerial API**: Check browser compatibility and HTTPS requirements
- **Device Communication**: Verify USB drivers and device connection
- **Protocol Implementation**: Compare with the Python reference implementation 