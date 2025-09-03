# Nextwaves RFID SDK for Python
@binhotvn - 3 Sep 2025

A comprehensive Python SDK for communicating with NRN RFID readers. This SDK provides a high-level interface for all major RFID operations including inventory, tag reading/writing, antenna control, and reader configuration.

## Features

- **Complete RFID Operations**: Inventory, tag reading/writing, antenna control
- **Reader Configuration**: Power settings, RF bands, baseband parameters
- **Professional Logging**: Configurable logging with different levels
- **Error Handling**: Comprehensive exception handling with custom SDK exceptions
- **Thread-Safe**: Safe for use in multi-threaded applications
- **Well Documented**: Extensive documentation and examples

## Installation

### Prerequisites

- Python 3.6 or higher
- pyserial library

```bash
pip install pyserial
```

### SDK Files

- `nrn.py` - Main SDK module
- `example.py` - Usage examples
- `README.md` - This documentation

## Quick Start

```python
from sdk.nation.nrn import create_reader
import logging

# Create reader instance
reader = create_reader("/dev/ttyUSB0", log_level=logging.INFO)

# Connect to reader
reader.open()

# Start inventory
def on_tag(tag):
    print(f"Tag: {tag['epc']}, RSSI: {tag['rssi']}")

reader.start_inventory_with_mode(antenna_mask=[1], callback=on_tag)

# Stop inventory
reader.stop_inventory()

# Close connection
reader.close()
```

## API Reference

### Main Classes

#### `NRNReader`

The main class for RFID reader operations.

```python
reader = NRNReader(port="/dev/ttyUSB0", baudrate=115200)
```

**Key Methods:**

- `open()` - Connect to the reader
- `close()` - Disconnect from the reader
- `start_inventory_with_mode()` - Start tag inventory
- `stop_inventory()` - Stop tag inventory
- `write_epc_tag_auto()` - Write EPC to a tag
- `configure_reader_power()` - Configure antenna power
- `set_rf_band()` - Set RF frequency band

#### `UARTConnection`

Handles low-level UART communication.

### Factory Function

#### `create_reader()`

Recommended way to create a configured reader instance:

```python
reader = create_reader(
    port="/dev/ttyUSB0",
    baudrate=115200,
    timeout=0.5,
    log_level=logging.INFO
)
```

### Logging

The SDK uses Python's standard logging module:

```python
import logging
from sdk.nation.nation import setup_logging

# Setup logging
logger = setup_logging(level=logging.DEBUG)

# Or set log level on existing reader
reader.set_log_level(logging.DEBUG)
```

### Error Handling

The SDK provides custom exceptions:

```python
from sdk.nation.nation import ConnectionError, ProtocolError, ConfigurationError

try:
    reader.open()
except ConnectionError as e:
    print(f"Connection failed: {e}")
```

## Examples

### Basic Inventory

```python
from sdk.nation.nrn import create_reader
import time

reader = create_reader("/dev/ttyUSB0")

def on_tag(tag):
    print(f"EPC: {tag['epc']}, RSSI: {tag['rssi']}")

reader.open()
reader.start_inventory_with_mode(antenna_mask=[1], callback=on_tag)
time.sleep(10)  # Run for 10 seconds
reader.stop_inventory()
reader.close()
```

### Tag Writing

```python
# Write new EPC to a tag
result = reader.write_epc_tag_auto(
    target_tag_epc="12345678",  # Current EPC
    new_epc_hex="87654321"      # New EPC
)

if result['success']:
    print("EPC written successfully")
else:
    print(f"Write failed: {result['result_msg']}")
```

### Reader Configuration

```python
# Configure antenna power
power_settings = {1: 30, 2: 25}  # Antenna 1: 30dBm, Antenna 2: 25dBm
reader.configure_reader_power(power_settings)

# Set RF band (FCC 902-928 MHz)
reader.set_rf_band(band_code=3)

# Configure baseband parameters
reader.configure_baseband(
    speed=0,      # Tari/Modulation
    q_value=4,    # Q value
    session=0,    # Session
    inventory_flag=0  # Inventory flag
)
```

### Reader Information

```python
# Get reader information
info = reader.Query_Reader_Information()
print(f"Serial: {info.get('serial_number')}")
print(f"App Version: {info.get('app_version')}")

# Get RFID capabilities
capabilities = reader.query_rfid_ability()
print(f"Max Power: {capabilities.get('max_power_dbm')} dBm")
print(f"Antenna Count: {capabilities.get('antenna_count')}")
```

## Configuration

### Supported RF Bands

- 0: CN 920–925 MHz
- 1: CN 840–845 MHz
- 2: CN Dual-band 840–845 + 920–925 MHz
- 3: FCC 902–928 MHz
- 4: ETSI 866–868 MHz
- 5: JP 916.8–920.4 MHz
- 6: TW 922.25–927.75 MHz
- 7: ID 923.125–925.125 MHz
- 8: RUS 866.6–867.4 MHz

### Power Settings

Power levels are specified in dBm (0-36 dBm typically).

### Antenna Configuration

Antennas are numbered 1-32. Use antenna masks to enable multiple antennas:

```python
# Enable antennas 1, 2, and 3
antenna_mask = [1, 2, 3]
reader.start_inventory_with_mode(antenna_mask=antenna_mask, callback=on_tag)
```

## Troubleshooting

### Common Issues

1. **Connection Failed**: Check port name and permissions
2. **No Tags Found**: Verify antenna connection and power settings
3. **Write Failed**: Ensure tag is in range and not locked

### Debug Mode

Enable debug logging for detailed information:

```python
reader.set_log_level(logging.DEBUG)
```

## SDK Information

- **Name**: Nextwaves RFID SDK
- **Version**: 1.0.0
- **Supported Readers**: NRN RFID Readers
- **Protocol**: UART-based communication
- **Python Version**: 3.6+

## License

This SDK is provided by Nextwaves for use with their RFID readers.

## Support

For technical support and documentation, please contact Nextwaves support.
