import serial
import time
from typing import List, Optional, Callable, Dict, Any
from dataclasses import dataclass
import logging
from serial.tools import list_ports
# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class MessageTran:
    """Message transaction class for handling reader communication"""
    packet_type: int = 160  # Default packet type (0xA0)
    data_len: int = 0
    read_id: int = 0
    cmd: int = 0
    data: List[int] = None
    check: int = 0
    tran_data: List[int] = None

    def __post_init__(self):
        if self.data is None:
            self.data = []
        if self.tran_data is None:
            self.tran_data = []

    @staticmethod
    def checksum(buffer: List[int], start_pos: int, length: int) -> int:
        """Calculate checksum for the message"""
        sum_val = sum(buffer[start_pos:start_pos + length])
        return (~sum_val + 1) & 0xff

    def create_message(self, read_id: int, cmd: int, data: Optional[List[int]] = None) -> List[int]:
        """Create a new message with the given parameters"""
        self.read_id = read_id
        self.cmd = cmd
        self.data = data or []
        self.data_len = len(self.data) + 3
        
        # Create base message
        self.tran_data = [self.packet_type, self.data_len, self.read_id, self.cmd]
        
        # Add data if present
        if self.data:
            self.tran_data.extend(self.data)
            
        # Calculate and add checksum
        self.check = self.checksum(self.tran_data, 0, len(self.tran_data))
        self.tran_data.append(self.check)
        
        return self.tran_data

class NextWavesReader:
    """Main reader class for Nextwaves VMR64 RFID reader"""
    
    def info(self):
        """Get reader information"""
        message = MessageTran()
        cmd = message.create_message(0x76, 0x01)
        return self.send_message(cmd)
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """Initialize the reader with serial port settings"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.buffer = bytearray()
        self.receive_callback = None
        self.send_callback = None
        self.analyze_callback = None
        
    def connect(self) -> bool:
        """Connect to the reader"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            logger.info(f"Connected to reader on port {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to reader: {e}")
            return False

    def disconnect(self):
        """Disconnect from the reader"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("Disconnected from reader")

    def send_message(self, message: List[int]) -> bool:
        """Send a message to the reader"""
        if not self.serial or not self.serial.is_open:
            logger.error("Reader not connected")
            return False
            
        try:
            self.serial.write(bytes(message))
            if self.send_callback:
                self.send_callback(message)
            return True
        except Exception as e:
            logger.error(f"Failed to send message: {e}")
            return False

    def read_data(self) -> Optional[List[int]]:
        """Read data from the reader"""
        if not self.serial or not self.serial.is_open:
            return None
            
        try:
            if self.serial.in_waiting:
                data = list(self.serial.read(self.serial.in_waiting))
                if self.receive_callback:
                    self.receive_callback(data)
                return data
        except Exception as e:
            logger.error(f"Failed to read data: {e}")
        return None

    def set_power(self, p1: int = 30, p2: int = 30, p3: int = 30, p4: int = 10) -> bool:
        """Set reader output power for all antennas"""
        if not all(0 <= p <= 33 for p in (p1, p2, p3, p4)):
            raise ValueError("Power values must be between 0-33 dBm")
            
        message = MessageTran()
        cmd = message.create_message(0x76, 0x07, [p1, p2, p3, p4])
        return self.send_message(cmd)

    def start_inventory(self, read_id: int = 0xFF) -> bool:
        """Start inventory operation"""
        message = MessageTran()
        cmd = message.create_message(read_id, 0x8A, [0x00, 0x01, 0x01, 0x01, 0x02, 0x01, 0x03, 0x01, 0x01, 0x01])
        return self.send_message(cmd)

    def stop_inventory(self, read_id: int = 0xFF) -> bool:
        """Stop inventory operation"""
        message = MessageTran()
        cmd = message.create_message(read_id, 0x8B)
        return self.send_message(cmd)

    def set_receive_callback(self, callback: Callable[[List[int]], None]):
        """Set callback for received data"""
        self.receive_callback = callback

    def set_send_callback(self, callback: Callable[[List[int]], None]):
        """Set callback for sent data"""
        self.send_callback = callback

    def set_analyze_callback(self, callback: Callable[[MessageTran], None]):
        """Set callback for analyzed data"""
        self.analyze_callback = callback

    def parse_inventory_data(self, data: List[int]) -> List[Dict[str, Any]]:
        """Parse inventory data into readable format"""
        tags = []
        i = 0
        
        while i + 2 <= len(data):
            if data[i] != 0xA0:
                i += 1
                continue
                
            length = data[i + 1]
            frame_len = length + 2
            
            if i + frame_len > len(data):
                break
                
            frame = data[i:i + frame_len]
            
            if len(frame) >= 5 and frame[3] == 0x8A:
                freq_ant = frame[4]
                antenna = (freq_ant & 0x03) + 1
                ch_idx = freq_ant >> 2
                
                pc_low = frame[5]
                epc_words = (pc_low >> 3) & 0x1F
                epc_len = epc_words * 2
                
                epc_start = 7
                epc_end = epc_start + epc_len
                
                if epc_end > len(frame) - 2:
                    epc_end = len(frame) - 2
                    
                epc = bytes(frame[epc_start:epc_end]).hex().upper()
                
                if epc:
                    rssi_byte = frame[-2]
                    rssi = -100 + rssi_byte  # Simple RSSI calculation
                    
                    tags.append({
                        "epc": epc,
                        "antenna": antenna,
                        "frequency": 865.0 + (ch_idx * 0.5),  # Frequency calculation
                        "rssi": rssi
                    })
                    
            i += frame_len
            
        return tags

# Example usage
if __name__ == "__main__":
    ports = list_ports.comports()
    for port in ports:
        print(port.device)
    # Create reader instance
    reader = NextWavesReader(port="/dev/cu.usbserial-0001")  # Change port 
    
    # Connect to reader
    if reader.connect():
        try:
            # Set power
            reader.set_power(30, 30, 30, 30)
            
            # Start inventory
            
            # Read loop
            while True:
                reader.start_inventory()

                data = reader.read_data()
                if data:
                    tags = reader.parse_inventory_data(data)
                    for tag in tags:
                        print(f"Tag: {tag}")
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\nStopping inventory...")
            reader.stop_inventory()
        finally:
            reader.disconnect()