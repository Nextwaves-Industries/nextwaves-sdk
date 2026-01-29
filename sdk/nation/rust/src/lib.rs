//! Nation RFID Reader SDK for Rust
//!
//! A comprehensive serial communication SDK for NRN RFID readers.
//!
//! # Example
//! ```no_run
//! use nrn_sdk::{NRNReader, TagData};
//!
//! fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let mut reader = NRNReader::new("/dev/ttyUSB0", 115200)?;
//!     reader.connect_and_initialize()?;
//!     
//!     reader.start_inventory(0x01, |tag| {
//!         println!("EPC: {}, RSSI: {:?} dBm", tag.epc, tag.rssi);
//!     })?;
//!     
//!     std::thread::sleep(std::time::Duration::from_secs(5));
//!     reader.stop_inventory()?;
//!     
//!     Ok(())
//! }
//! ```

use byteorder::{BigEndian, ByteOrder};
use log::{debug, error, info, warn};
use serialport::SerialPort;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use thiserror::Error;

// === Constants ===

pub const FRAME_HEADER: u8 = 0x5A;
const CRC16_CCITT_INIT: u16 = 0x0000;
const CRC16_CCITT_POLY: u16 = 0x8005;

// === Message IDs ===

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum MID {
    // Reader Configuration
    QueryInfo = 0x0100,
    ConfirmConnection = 0x12,

    // RFID Inventory
    ReadEpcTag = 0x0210,
    PhaseInventory = 0x0214,
    StopInventory = 0x02FF,
    WriteEpcTag = 0x0211,

    // Error Handling
    ErrorNotification = 0x00,

    // RFID Baseband
    ConfigBaseband = 0x020B,
    QueryBaseband = 0x020C,

    // Power Control
    ConfigureReaderPower = 0x0201,
    QueryReaderPower = 0x0202,
    ReaderPowerCalibration = 0x0103,
    QueryPowerCalibration = 0x0104,

    // Filter Settings
    SetFilterSettings = 0x0209,
    QueryFilterSettings = 0x020A,

    // RF Band & Frequency
    SetRfBand = 0x0203,
    QueryRfBand = 0x0204,
    SetWorkingFrequency = 0x0205,
    QueryWorkingFrequency = 0x0206,

    // RFID Ability
    QueryRfidAbility = 0x1000,

    // Buzzer Control
    BuzzerSwitch = 0x011E,

    // GPIO Commands
    ConfigureGpo = 0x0109,
    QueryGpi = 0x010A,
    ConfigureGpiTrigger = 0x010B,
    QueryGpiTrigger = 0x010C,
}

impl From<MID> for u16 {
    fn from(mid: MID) -> u16 {
        mid as u16
    }
}

// === Beeper Modes ===

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BeeperMode {
    Quiet = 0x00,
    BeepAfterInventory = 0x01,
    BeepAfterTag = 0x02,
}

// === RF Profiles ===

#[derive(Debug, Clone)]
pub struct RFProfile {
    pub id: u8,
    pub name: &'static str,
    pub description: &'static str,
}

pub const RF_PROFILES: [RFProfile; 3] = [
    RFProfile {
        id: 0,
        name: "Profile 0",
        description: "Default baseband profile",
    },
    RFProfile {
        id: 1,
        name: "Profile 1",
        description: "High performance profile",
    },
    RFProfile {
        id: 2,
        name: "Profile 2",
        description: "Dense tag profile",
    },
];

// === Error Types ===

#[derive(Error, Debug)]
pub enum NRNError {
    #[error("Serial port error: {0}")]
    SerialError(#[from] serialport::Error),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Connection error: {0}")]
    ConnectionError(String),

    #[error("Protocol error: {0}")]
    ProtocolError(String),

    #[error("Timeout error")]
    TimeoutError,

    #[error("CRC mismatch")]
    CrcError,
}

pub type Result<T> = std::result::Result<T, NRNError>;

// === Data Structures ===

#[derive(Debug, Clone, Default)]
pub struct TagData {
    pub epc: String,
    pub pc: String,
    pub antenna_id: u8,
    pub rssi: Option<i32>,
    pub tid: Option<String>,
    pub phase: Option<f64>,
    pub frequency: Option<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct ReaderInfo {
    pub serial_number: String,
    pub power_on_time_sec: u32,
    pub baseband_compile_time: String,
    pub app_version: String,
    pub os_version: String,
    pub app_compile_time: String,
}

#[derive(Debug, Clone, Default)]
pub struct RFIDAbility {
    pub min_power: i32,
    pub max_power: i32,
    pub antenna_count: u8,
    pub frequencies: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct ParsedFrame {
    pub valid: bool,
    pub proto_type: u8,
    pub proto_ver: u8,
    pub rs485: bool,
    pub notify: bool,
    pub category: u8,
    pub mid: u8,
    pub data_length: u16,
    pub data: Vec<u8>,
    pub crc: u16,
}

// === Utility Functions ===

/// Calculate CRC16-CCITT checksum
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = CRC16_CCITT_INIT;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ CRC16_CCITT_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Calculate RSSI in dBm from raw byte value
/// Formula: -100 + round((rssiRaw * 70) / 255)
pub fn calculate_rssi(rssi_raw: u8) -> i32 {
    -100 + ((rssi_raw as i32 * 70 + 127) / 255)
}

/// Calculate frequency in MHz from channel index
/// Formula: 920.0 + chIdx * 0.5
pub fn calculate_frequency(ch_idx: u8) -> f64 {
    920.0 + (ch_idx as f64) * 0.5
}

/// Convert bytes to hex string
pub fn bytes_to_hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{:02X}", b)).collect()
}

/// Build antenna mask from list of antenna IDs
pub fn build_antenna_mask(antennas: &[u8]) -> u32 {
    let mut mask: u32 = 0;
    for &ant_id in antennas {
        if ant_id >= 1 && ant_id <= 32 {
            mask |= 1 << (ant_id - 1);
        }
    }
    mask
}

// === NRNReader ===

pub struct NRNReader {
    port: Box<dyn SerialPort>,
    antenna_mask: u32,
    running: Arc<AtomicBool>,
}

impl NRNReader {
    /// Create a new NRNReader instance
    ///
    /// # Arguments
    /// * `port_name` - Serial port path (e.g., "/dev/ttyUSB0", "COM3")
    /// * `baudrate` - Baud rate (typically 115200)
    pub fn new(port_name: &str, baudrate: u32) -> Result<Self> {
        let port = serialport::new(port_name, baudrate)
            .timeout(Duration::from_millis(500))
            .open()?;

        info!("Opened port: {}", port_name);

        Ok(Self {
            port,
            antenna_mask: 0x00000001,
            running: Arc::new(AtomicBool::new(false)),
        })
    }

    /// Connect and initialize the reader
    pub fn connect_and_initialize(&mut self) -> Result<()> {
        let frame = self.build_frame(MID::ConfirmConnection, &[]);
        self.port.write_all(&frame)?;

        info!("Reader initialized");
        Ok(())
    }

    /// Build EPC read payload with optional TID reading
    pub fn build_epc_read_payload(
        &self,
        antenna_mask: u32,
        continuous: bool,
        include_tid: bool,
    ) -> Vec<u8> {
        let mask = if antenna_mask == 0 {
            0x00000001
        } else {
            antenna_mask
        };

        let mut data = vec![
            ((mask >> 24) & 0xFF) as u8,
            ((mask >> 16) & 0xFF) as u8,
            ((mask >> 8) & 0xFF) as u8,
            (mask & 0xFF) as u8,
            if continuous { 0x01 } else { 0x00 },
        ];

        if include_tid {
            data.push(0x02); // PID 0x02: TID reading parameters
            data.push(0x00); // Byte 0: TID reading mode (0: self-adaptable)
            data.push(0x00); // Byte 1: Word length (0x00 = Auto / Max)
        }

        data
    }

    /// Build a protocol frame
    pub fn build_frame(&self, mid: MID, payload: &[u8]) -> Vec<u8> {
        let mid_val: u16 = mid.into();
        let category = ((mid_val >> 8) & 0xFF) as u8;
        let mid_byte = (mid_val & 0xFF) as u8;

        let mut frame = vec![FRAME_HEADER];

        // PCW: Proto Type, Proto Version, Flags, Category
        frame.push(0x00); // Proto Type
        frame.push(0x01); // Proto Version
        frame.push(0x00); // Flags
        frame.push(category);

        // MID
        frame.push(mid_byte);

        // Length
        let len = payload.len() as u16;
        frame.push(((len >> 8) & 0xFF) as u8);
        frame.push((len & 0xFF) as u8);

        // Payload
        frame.extend_from_slice(payload);

        // CRC
        let crc = crc16_ccitt(&frame[1..]);
        frame.push(((crc >> 8) & 0xFF) as u8);
        frame.push((crc & 0xFF) as u8);

        frame
    }

    /// Parse EPC tag data from inventory response
    pub fn parse_epc(&self, data: &[u8]) -> TagData {
        let mut tag = TagData::default();

        if data.len() < 5 {
            return tag;
        }

        // 1. Read EPC (Mandatory)
        let epc_len = BigEndian::read_u16(&data[0..2]) as usize;
        if data.len() < 2 + epc_len + 3 {
            return tag;
        }

        tag.epc = bytes_to_hex(&data[2..2 + epc_len]);

        // 2. Read PC (Mandatory - 2 bytes)
        tag.pc = bytes_to_hex(&data[2 + epc_len..2 + epc_len + 2]);

        // 3. Read Antenna ID (Mandatory - 1 byte)
        tag.antenna_id = data[2 + epc_len + 2];

        // 4. Parse optional PIDs
        let mut cursor = 2 + epc_len + 3;

        while cursor < data.len() {
            let pid = data[cursor];
            cursor += 1;

            match pid {
                0x01 => {
                    // RSSI (1 byte)
                    if cursor < data.len() {
                        tag.rssi = Some(calculate_rssi(data[cursor]));
                        cursor += 1;
                    } else {
                        break;
                    }
                }
                0x02 => {
                    // Reading Result (1 byte)
                    if cursor < data.len() {
                        cursor += 1;
                    } else {
                        break;
                    }
                }
                0x03 => {
                    // TID DATA (Variable length)
                    if cursor + 1 < data.len() {
                        let tid_len = BigEndian::read_u16(&data[cursor..cursor + 2]) as usize;
                        cursor += 2;
                        if cursor + tid_len <= data.len() {
                            tag.tid = Some(bytes_to_hex(&data[cursor..cursor + tid_len]));
                            cursor += tid_len;
                        } else {
                            break;
                        }
                    } else {
                        break;
                    }
                }
                0x04 | 0x05 => {
                    // Tag/Reserve data
                    if cursor + 1 < data.len() {
                        let byte_len = BigEndian::read_u16(&data[cursor..cursor + 2]) as usize;
                        cursor += 2 + byte_len;
                    } else {
                        break;
                    }
                }
                0x06 => cursor += 1,  // Sub-antenna
                0x07 => cursor += 8,  // UTC time
                0x08 => {
                    // Frequency (4 bytes, KHz)
                    if cursor + 3 < data.len() {
                        let freq_khz = BigEndian::read_u32(&data[cursor..cursor + 4]);
                        tag.frequency = Some(freq_khz as f64 / 1000.0);
                        cursor += 4;
                    } else {
                        break;
                    }
                }
                0x09 => {
                    // Phase (1 byte, 0~128)
                    if cursor < data.len() {
                        let phase_raw = data[cursor];
                        tag.phase = Some((phase_raw as f64 / 128.0) * 2.0 * std::f64::consts::PI);
                        cursor += 1;
                    } else {
                        break;
                    }
                }
                _ => continue,
            }
        }

        tag
    }

    /// Start continuous inventory
    pub fn start_inventory<F>(&mut self, antenna_mask: u32, _callback: F) -> Result<()>
    where
        F: Fn(TagData) + Send + 'static,
    {
        self.antenna_mask = antenna_mask;
        self.running.store(true, Ordering::SeqCst);

        let payload = self.build_epc_read_payload(antenna_mask, true, false);
        let frame = self.build_frame(MID::ReadEpcTag, &payload);
        self.port.write_all(&frame)?;

        info!("Inventory started with mask: 0x{:08X}", antenna_mask);
        Ok(())
    }

    /// Stop current inventory operation
    pub fn stop_inventory(&mut self) -> Result<()> {
        self.running.store(false, Ordering::SeqCst);

        let frame = self.build_frame(MID::StopInventory, &[]);
        self.port.write_all(&frame)?;

        info!("Inventory stopped");
        Ok(())
    }

    /// Configure antenna power
    pub fn configure_power(&mut self, powers: &HashMap<u8, u8>, persist: bool) -> Result<()> {
        let mut payload = vec![if persist { 0x01 } else { 0x00 }];

        for (&ant_id, &power) in powers {
            payload.push(ant_id);
            payload.push(power);
        }

        let frame = self.build_frame(MID::ConfigureReaderPower, &payload);
        self.port.write_all(&frame)?;

        info!("Power configured");
        Ok(())
    }

    /// Query current power settings
    pub fn query_power(&mut self) -> Result<HashMap<u8, u8>> {
        let frame = self.build_frame(MID::QueryReaderPower, &[]);
        let response = self.send_and_receive(&frame)?;

        let mut powers = HashMap::new();
        // Response format: [persistence byte] + [ant_id, power] pairs
        if response.len() >= 1 {
            let mut i = 1;
            while i + 1 < response.len() {
                let ant_id = response[i];
                let power = response[i + 1];
                powers.insert(ant_id, power);
                i += 2;
            }
        }

        // Default if empty response
        if powers.is_empty() {
            powers = HashMap::from([(1, 30), (2, 30), (3, 30), (4, 30)]);
        }

        Ok(powers)
    }

    /// Configure GPO state
    pub fn configure_gpo(&mut self, gpo_id: u8, state: bool) -> Result<()> {
        let payload = vec![gpo_id, if state { 0x01 } else { 0x00 }];
        let frame = self.build_frame(MID::ConfigureGpo, &payload);
        self.port.write_all(&frame)?;
        Ok(())
    }

    /// Query GPI state
    pub fn query_gpi(&mut self, gpi_id: u8) -> Result<bool> {
        let frame = self.build_frame(MID::QueryGpi, &[gpi_id]);
        let response = self.send_and_receive(&frame)?;

        // Response format: [gpi_id, state]
        if response.len() >= 2 {
            return Ok(response[1] != 0x00);
        }

        Ok(false)
    }

    /// Query reader information
    pub fn query_reader_information(&mut self) -> Result<ReaderInfo> {
        let frame = self.build_frame(MID::QueryInfo, &[]);
        let response = self.send_and_receive(&frame)?;

        let mut info = ReaderInfo::default();

        // Parse TLV-style response data
        let mut cursor = 0;
        while cursor < response.len() {
            if cursor + 2 > response.len() {
                break;
            }

            let pid = response[cursor];
            let plen = response[cursor + 1] as usize;
            cursor += 2;

            if cursor + plen > response.len() {
                break;
            }

            let field_data = &response[cursor..cursor + plen];
            cursor += plen;

            match pid {
                0x01 => info.serial_number = String::from_utf8_lossy(field_data).to_string(),
                0x02 if plen >= 4 => info.power_on_time_sec = BigEndian::read_u32(field_data),
                0x03 => info.baseband_compile_time = String::from_utf8_lossy(field_data).to_string(),
                0x04 => info.app_version = String::from_utf8_lossy(field_data).to_string(),
                0x05 => info.os_version = String::from_utf8_lossy(field_data).to_string(),
                0x06 => info.app_compile_time = String::from_utf8_lossy(field_data).to_string(),
                _ => {}
            }
        }

        Ok(info)
    }

    /// Query RFID capabilities
    pub fn query_rfid_ability(&mut self) -> Result<RFIDAbility> {
        let frame = self.build_frame(MID::QueryRfidAbility, &[]);
        let response = self.send_and_receive(&frame)?;

        let mut ability = RFIDAbility {
            min_power: 0,
            max_power: 33,
            antenna_count: 4,
            frequencies: vec![],
        };

        // Parse TLV-style response data
        let mut cursor = 0;
        while cursor < response.len() {
            if cursor + 2 > response.len() {
                break;
            }

            let pid = response[cursor];
            let plen = response[cursor + 1] as usize;
            cursor += 2;

            if cursor + plen > response.len() {
                break;
            }

            let field_data = &response[cursor..cursor + plen];
            cursor += plen;

            match pid {
                0x01 if plen >= 1 => ability.min_power = field_data[0] as i32,
                0x02 if plen >= 1 => ability.max_power = field_data[0] as i32,
                0x03 if plen >= 1 => ability.antenna_count = field_data[0],
                0x04 => {
                    let mut i = 0;
                    while i + 3 < plen {
                        let freq_khz = BigEndian::read_u32(&field_data[i..i + 4]);
                        ability.frequencies.push(freq_khz as f64 / 1000.0);
                        i += 4;
                    }
                }
                _ => {}
            }
        }

        Ok(ability)
    }

    /// Send a frame and wait for response
    fn send_and_receive(&mut self, frame: &[u8]) -> Result<Vec<u8>> {
        // Clear input buffer
        self.port.clear(serialport::ClearBuffer::Input)?;

        // Send frame
        self.port.write_all(frame)?;

        // Read response
        let mut buffer = vec![0u8; 1024];
        let n = self.port.read(&mut buffer)?;

        if n < 9 {
            return Err(NRNError::ProtocolError(format!(
                "Response too short: {} bytes",
                n
            )));
        }

        // Parse frame
        let parsed = self.parse_frame(&buffer[..n]);
        if !parsed.valid {
            return Err(NRNError::ProtocolError("Invalid response frame".into()));
        }

        Ok(parsed.data)
    }

    /// Parse a raw protocol frame
    pub fn parse_frame(&self, data: &[u8]) -> ParsedFrame {
        let mut frame = ParsedFrame {
            valid: false,
            proto_type: 0,
            proto_ver: 0,
            rs485: false,
            notify: false,
            category: 0,
            mid: 0,
            data_length: 0,
            data: vec![],
            crc: 0,
        };

        if data.len() < 9 || data[0] != FRAME_HEADER {
            return frame;
        }

        frame.proto_type = data[1];
        frame.proto_ver = data[2];
        frame.rs485 = (data[3] & 0x80) != 0;
        frame.notify = (data[3] & 0x10) != 0;
        frame.category = data[4];
        frame.mid = data[5];
        frame.data_length = BigEndian::read_u16(&data[6..8]);

        let expected_len = 8 + frame.data_length as usize + 2;
        if data.len() < expected_len {
            return frame;
        }

        frame.data = data[8..8 + frame.data_length as usize].to_vec();

        let crc_offset = 8 + frame.data_length as usize;
        frame.crc = BigEndian::read_u16(&data[crc_offset..crc_offset + 2]);

        // Verify CRC
        let calculated_crc = crc16_ccitt(&data[1..crc_offset]);
        frame.valid = calculated_crc == frame.crc;

        frame
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_rssi() {
        assert_eq!(calculate_rssi(0), -100);
        assert_eq!(calculate_rssi(255), -30);
        assert_eq!(calculate_rssi(128), -65);
    }

    #[test]
    fn test_calculate_frequency() {
        assert_eq!(calculate_frequency(0), 920.0);
        assert_eq!(calculate_frequency(10), 925.0);
    }

    #[test]
    fn test_build_antenna_mask() {
        assert_eq!(build_antenna_mask(&[1]), 0x00000001);
        assert_eq!(build_antenna_mask(&[1, 2]), 0x00000003);
        assert_eq!(build_antenna_mask(&[1, 2, 3, 4]), 0x0000000F);
    }

    #[test]
    fn test_bytes_to_hex() {
        assert_eq!(bytes_to_hex(&[0xDE, 0xAD, 0xBE, 0xEF]), "DEADBEEF");
    }

    #[test]
    fn test_crc16() {
        let data = vec![0x00, 0x01, 0x00, 0x02, 0x10, 0x00, 0x05];
        let crc = crc16_ccitt(&data);
        assert!(crc > 0);
    }
}
