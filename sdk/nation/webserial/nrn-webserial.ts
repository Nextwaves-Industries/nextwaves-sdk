/**
 * NRN WebSerial SDK for TypeScript
 * 
 * A comprehensive Web Serial API SDK for communicating with NRN RFID readers.
 * This SDK provides a high-level interface for all major RFID operations including
 * inventory, tag reading/writing, antenna control, and reader configuration.
 * 
 * @version 1.0.0
 * @author Nextwaves
 */

// === Web Serial API Type Declarations ===
// These declarations extend the Navigator interface for Web Serial API support
declare global {
  interface Navigator {
    serial: Serial;
  }

  interface Serial {
    requestPort(options?: SerialPortRequestOptions): Promise<SerialPort>;
    getPorts(): Promise<SerialPort[]>;
  }

  interface SerialPortRequestOptions {
    filters?: SerialPortFilter[];
  }

  interface SerialPortFilter {
    usbVendorId?: number;
    usbProductId?: number;
  }

  interface SerialPort {
    readable: ReadableStream<Uint8Array> | null;
    writable: WritableStream<Uint8Array> | null;
    open(options: SerialOptions): Promise<void>;
    close(): Promise<void>;
    getInfo(): SerialPortInfo;
  }

  interface SerialOptions {
    baudRate: number;
    dataBits?: number;
    stopBits?: number;
    parity?: 'none' | 'even' | 'odd';
    bufferSize?: number;
    flowControl?: 'none' | 'hardware';
  }

  interface SerialPortInfo {
    usbVendorId?: number;
    usbProductId?: number;
  }
}

// === Type Definitions ===
export interface NRNWebSerialOptions {
  baudrate?: number;
  timeout?: number;
  rs485?: boolean;
  onLog?: (level: string, message: string) => void;
}

export interface SDKInfo {
  name: string;
  version: string;
  description: string;
}

export interface ReaderInfo {
  baudrate: number;
  timeout: number;
  rs485: boolean;
  isConnected: boolean;
  antennaMask: string;
}

export interface ParsedFrame {
  valid: boolean;
  type: 'notification' | 'response';
  protoType: number;
  protoVer: number;
  rs485: boolean;
  notify: boolean;
  pcw: number;
  category: number;
  mid: number;
  address: number | null;
  dataLength: number;
  data: Uint8Array;
  crc: number;
  raw: Uint8Array;
}

export interface TagData {
  epc: string;
  pc: string;
  antenna_id: number;
  rssi: number | null;
  tid?: string;
  phase?: number;
  frequency?: number;
  error?: string;
}

export interface ReaderInformation {
  serial_number?: string;
  power_on_time_sec?: number;
  baseband_compile_time?: string;
  app_version?: string;
  os_version?: string;
  app_compile_time?: string;
  error?: string;
}

export type TagCallback = (tag: TagData) => void;
export type LogCallback = (level: string, message: string) => void;

// === SDK Configuration ===
export const SDK_NAME = "Nextwaves NRN WebSerial SDK";
export const SDK_VERSION = "1.0.0";

// === Protocol Constants ===
export const CRC16_CCITT_INIT = 0x0000;
export const CRC16_CCITT_POLY = 0x8005;

export const FRAME_HEADER = 0x5A;
export const PROTO_TYPE = 0x00;
export const PROTO_VER = 0x01;
export const RS485_FLAG = 0x00;
export const READER_NOTIFY_FLAG = 0x00;

// === Message ID Constants ===
export const MID = {
  // Reader Configuration
  QUERY_INFO: 0x0100,
  CONFIRM_CONNECTION: 0x12,

  // RFID Inventory
  READ_EPC_TAG: (0x02 << 8) | 0x10,
  PHASE_INVENTORY: 0x0214,
  STOP_INVENTORY: (0x02 << 8) | 0xFF,
  STOP_OPERATION: 0xFF,
  READ_END: 0x1231,
  WRITE_EPC_TAG: 0x0211,

  // Error Handling
  ERROR_NOTIFICATION: 0x00,

  // RFID Baseband
  CONFIG_BASEBAND: 0x020B,
  QUERY_BASEBAND: 0x020C,

  // Session Management
  SESSION: 0x03,

  // Power Control
  CONFIGURE_READER_POWER: 0x0201,
  QUERY_READER_POWER: 0x0202,
  READER_POWER_CALIBRATION: 0x0103,
  QUERY_POWER_CALIBRATION: 0x0104,

  // Filter Settings
  SET_FILTER_SETTINGS: 0x0209,
  QUERY_FILTER_SETTINGS: 0x020a,

  // RF Band & Frequency
  SET_RF_BAND: 0x0203,
  QUERY_RF_BAND: 0x0204,
  SET_WORKING_FREQUENCY: 0x0205,
  QUERY_WORKING_FREQUENCY: 0x0206,

  // RFID Ability
  QUERY_RFID_ABILITY: 0x1000,

  // Antenna Control
  CONFIGURE_ANTENNA_ENABLE: 0x0203,
  QUERY_ANTENNA_ENABLE: 0x0202,

  // Buzzer Control
  BUZZER_SWITCH: (0x01 << 8) | 0x1E,

  // GPIO Commands (Category 1)
  CONFIGURE_GPO: 0x0109,
  QUERY_GPI: 0x010a,
  CONFIGURE_GPI_TRIGGER: 0x010b,
  QUERY_GPI_TRIGGER: 0x010c,
} as const;

// === Beeper Modes ===
export const BEEPER_MODES = {
  QUIET: 0x00,
  BEEP_AFTER_INVENTORY: 0x01,
  BEEP_AFTER_TAG: 0x02,
} as const;

// === RF Profiles ===
export const RF_PROFILES = {
  0: { id: 0, name: "Profile 0", description: "Default baseband profile" },
  1: { id: 1, name: "Profile 1", description: "High performance profile" },
  2: { id: 2, name: "Profile 2", description: "Dense tag profile" },
} as const;

// === SDK Exceptions ===
export class NRNWebSerialError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'NRNWebSerialError';
  }
}

export class ConnectionError extends NRNWebSerialError {
  constructor(message: string) {
    super(message);
    this.name = 'ConnectionError';
  }
}

export class ProtocolError extends NRNWebSerialError {
  constructor(message: string) {
    super(message);
    this.name = 'ProtocolError';
  }
}

export class ConfigurationError extends NRNWebSerialError {
  constructor(message: string) {
    super(message);
    this.name = 'ConfigurationError';
  }
}

export class TagOperationError extends NRNWebSerialError {
  constructor(message: string) {
    super(message);
    this.name = 'TagOperationError';
  }
}

// === Utility Functions ===
export class NRNUtils {
  /**
   * Calculate CRC16-CCITT checksum for data validation
   */
  static crc16CCITT(data: Uint8Array): number {
    let crc = 0x0000;

    for (const byte of data) {
      crc ^= byte << 8;
      for (let i = 0; i < 8; i++) {
        if (crc & 0x8000) {
          crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
        } else {
          crc = (crc << 1) & 0xFFFF;
        }
      }
    }
    return crc;
  }

  /**
   * Build Protocol Control Word (PCW) for frame header
   */
  static buildPCW(category: number, mid: number, rs485: boolean = false, notify: boolean = false): number {
    let pcw = (PROTO_TYPE << 24) | (PROTO_VER << 16);
    if (rs485) pcw |= (1 << 13);
    if (notify) pcw |= (1 << 12);
    pcw |= (category << 8) | mid;
    return pcw;
  }

  /**
   * Build a complete protocol frame for communication
   */
  static buildFrame(mid: number, payload: Uint8Array = new Uint8Array(0), rs485: boolean = false, notify: boolean = false): Uint8Array {
    const frameHeader = new Uint8Array([FRAME_HEADER]);
    const midValue = typeof mid === 'object' ? (mid as any).value : mid;
    const category = (midValue >> 8) & 0xFF;
    const midCode = midValue & 0xFF;

    const pcw = this.buildPCW(category, midCode, rs485, notify);
    const pcwBytes = new Uint8Array([
      (pcw >> 24) & 0xFF,
      (pcw >> 16) & 0xFF,
      (pcw >> 8) & 0xFF,
      pcw & 0xFF
    ]);

    const addrBytes = rs485 ? new Uint8Array([0x00]) : new Uint8Array(0);
    const lengthBytes = new Uint8Array([
      (payload.length >> 8) & 0xFF,
      payload.length & 0xFF
    ]);

    const frameContent = new Uint8Array([
      ...pcwBytes,
      ...addrBytes,
      ...lengthBytes,
      ...payload
    ]);

    const crcBytes = new Uint8Array([
      (this.crc16CCITT(frameContent) >> 8) & 0xFF,
      this.crc16CCITT(frameContent) & 0xFF
    ]);

    return new Uint8Array([...frameHeader, ...frameContent, ...crcBytes]);
  }

  /**
   * Parse a received frame from the RFID reader
   */
  static parseFrame(raw: Uint8Array): ParsedFrame {
    if (raw.length < 9) {
      throw new ProtocolError("Frame too short");
    }

    if (raw[0] !== FRAME_HEADER) {
      throw new ProtocolError("Invalid frame header");
    }

    let offset = 1; // skip header

    // Protocol Control Word
    const pcw = (raw[offset] << 24) | (raw[offset + 1] << 16) |
      (raw[offset + 2] << 8) | raw[offset + 3];
    offset += 4;

    const protoType = (pcw >> 24) & 0xFF;
    const protoVer = (pcw >> 16) & 0xFF;
    const rs485Flag = (pcw >> 13) & 0x01;
    const notifyFlag = (pcw >> 12) & 0x01;
    const category = (pcw >> 8) & 0xFF;
    const mid = pcw & 0xFF;
    const responseType: 'notification' | 'response' = notifyFlag ? "notification" : "response";

    // Optional Serial Address
    let addr: number | null = null;
    if (rs485Flag) {
      addr = raw[offset];
      offset += 1;
    }

    // Data Length
    const dataLen = (raw[offset] << 8) | raw[offset + 1];
    offset += 2;

    if (offset + dataLen + 2 > raw.length) {
      throw new ProtocolError("Frame length mismatch or truncated");
    }

    // Data
    const data = raw.slice(offset, offset + dataLen);
    offset += dataLen;

    // CRC
    const receivedCRC = (raw[offset] << 8) | raw[offset + 1];
    const calculatedCRC = this.crc16CCITT(raw.slice(1, offset));

    if (receivedCRC !== calculatedCRC) {
      throw new ProtocolError(`CRC mismatch! Got 0x${receivedCRC.toString(16).padStart(4, '0')}, expected 0x${calculatedCRC.toString(16).padStart(4, '0')}`);
    }

    return {
      valid: true,
      type: responseType,
      protoType,
      protoVer,
      rs485: Boolean(rs485Flag),
      notify: Boolean(notifyFlag),
      pcw,
      category,
      mid,
      address: addr,
      dataLength: dataLen,
      data,
      crc: receivedCRC,
      raw
    };
  }

  /**
   * Extract valid protocol frames from raw byte stream
   */
  static extractValidFrames(data: Uint8Array, rs485: boolean = false): Uint8Array[] {
    const frames: Uint8Array[] = [];
    let i = 0;

    while (i < data.length) {
      if (data[i] !== 0x5A) {
        i++;
        continue;
      }

      // Minimum valid length
      if (i + 9 > data.length) break;

      const length = (data[i + 5] << 8) | data[i + 6];
      const addrLen = rs485 ? 1 : 0;
      const fullLen = 1 + 4 + addrLen + 2 + length + 2;

      if (i + fullLen > data.length) break;

      const frame = data.slice(i, i + fullLen);
      const crcCalc = this.crc16CCITT(frame.slice(1, -2));
      const crcRecv = (frame[frame.length - 2] << 8) | frame[frame.length - 1];

      if (crcCalc === crcRecv) {
        frames.push(frame);
      } else {
        console.warn(`CRC mismatch at index ${i}: expected=0x${crcCalc.toString(16)}, got=0x${crcRecv.toString(16)}`);
      }

      i += fullLen;
    }

    return frames;
  }

  /**
   * Calculate RSSI in dBm from raw byte value
   * Formula aligned with protocols.ts NATION protocol
   */
  static calculateRSSI(rssiRaw: number): number {
    return -100 + Math.round((rssiRaw * 70) / 255);
  }

  /**
   * Calculate frequency in MHz from channel index
   * Formula aligned with protocols.ts NATION protocol
   */
  static calculateFrequency(chIdx: number): number {
    return 920.0 + chIdx * 0.5;
  }
}

// === Main NRN WebSerial Class ===
export class NRNWebSerial {
  private baudrate: number;
  private timeout: number;
  private rs485: boolean;
  private onLog: LogCallback;

  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
  private isConnected: boolean = false;
  private isInventoryRunning: boolean = false;
  private inventoryCallback: TagCallback | null = null;
  private antennaMask: number = 0x00000001; // Default to Main Antenna 1

  /**
   * Initialize the NRN WebSerial SDK
   */
  constructor(options: NRNWebSerialOptions = {}) {
    this.baudrate = options.baudrate || 115200;
    this.timeout = options.timeout || 500;
    this.rs485 = options.rs485 || false;
    this.onLog = options.onLog || console.log;

    this.log('info', `Initialized ${SDK_NAME} v${SDK_VERSION}`);
  }

  /**
   * Log message with level
   */
  private log(level: string, message: string): void {
    const timestamp = new Date().toISOString();
    this.onLog(level, `[${timestamp}] [${level.toUpperCase()}] ${message}`);
  }

  /**
   * Get SDK information
   */
  static getSDKInfo(): SDKInfo {
    return {
      name: SDK_NAME,
      version: SDK_VERSION,
      description: "Nextwaves NRN RFID Reader WebSerial SDK for TypeScript"
    };
  }

  /**
   * Get current reader configuration information
   */
  getReaderInfo(): ReaderInfo {
    return {
      baudrate: this.baudrate,
      timeout: this.timeout,
      rs485: this.rs485,
      isConnected: this.isConnected,
      antennaMask: `0x${this.antennaMask.toString(16).padStart(8, '0')}`
    };
  }

  /**
   * Check if Web Serial API is supported
   */
  static isSupported(): boolean {
    return 'serial' in navigator;
  }

  /**
   * Request port access and connect to the reader
   */
  async connect(): Promise<boolean> {
    if (!NRNWebSerial.isSupported()) {
      throw new ConnectionError("Web Serial API is not supported in this browser");
    }

    try {
      this.log('info', 'Requesting port access...');
      this.port = await navigator.serial.requestPort();

      this.log('info', 'Opening port...');
      await this.port.open({
        baudRate: this.baudrate,
        dataBits: 8,
        parity: 'none',
        stopBits: 1,
        flowControl: 'none'
      });

      this.reader = this.port.readable!.getReader();
      this.writer = this.port.writable!.getWriter();
      this.isConnected = true;

      this.log('info', `Connected to NRN reader @ ${this.baudrate}bps`);

      // Initialize reader
      await this.initializeReader();

      return true;
    } catch (error) {
      this.log('error', `Connection failed: ${(error as Error).message}`);
      throw new ConnectionError(`Connection failed: ${(error as Error).message}`);
    }
  }

  /**
   * Disconnect from the reader
   */
  async disconnect(): Promise<void> {
    try {
      if (this.isInventoryRunning) {
        await this.stopInventory();
      }

      if (this.reader) {
        await this.reader.releaseLock();
        this.reader = null;
      }

      if (this.writer) {
        await this.writer.releaseLock();
        this.writer = null;
      }

      if (this.port) {
        await this.port.close();
        this.port = null;
      }

      this.isConnected = false;
      this.log('info', 'Disconnected from NRN reader');
    } catch (error) {
      this.log('error', `Disconnect error: ${(error as Error).message}`);
    }
  }

  /**
   * Send data to the reader
   */
  async send(data: Uint8Array): Promise<void> {
    if (!this.isConnected || !this.writer) {
      throw new ConnectionError("Not connected to reader");
    }

    try {
      await this.writer.write(data);
      this.log('debug', `Sent ${data.length} bytes: ${Array.from(data).map((b: number) => b.toString(16).padStart(2, '0')).join(' ')}`);
    } catch (error) {
      this.log('error', `Send error: ${(error as Error).message}`);
      throw new ConnectionError(`Send error: ${(error as Error).message}`);
    }
  }

  /**
   * Receive data from the reader
   */
  async receive(): Promise<Uint8Array> {
    if (!this.isConnected || !this.reader) {
      throw new ConnectionError("Not connected to reader");
    }

    try {
      const { value, done } = await this.reader.read();
      if (done) {
        throw new ConnectionError("Reader stream ended");
      }

      this.log('debug', `Received ${value.length} bytes: ${Array.from(value).map((b: number) => b.toString(16).padStart(2, '0')).join(' ')}`);
      return value;
    } catch (error) {
      this.log('error', `Receive error: ${(error as Error).message}`);
      throw new ConnectionError(`Receive error: ${(error as Error).message}`);
    }
  }

  /**
   * Initialize the reader
   */
  private async initializeReader(): Promise<boolean> {
    try {
      this.log('info', 'Initializing reader...');

      // Send STOP command to ensure idle state
      const stopFrame = NRNUtils.buildFrame(MID.STOP_INVENTORY, new Uint8Array(0));
      await this.send(stopFrame);

      await this.delay(100);
      const raw = await this.receive();

      if (!raw || raw.length === 0) {
        this.log('warn', 'No response received during initialization');
        return false;
      }

      const frame = NRNUtils.parseFrame(raw);
      this.log('debug', `MID: 0x${frame.mid.toString(16)}, Data: ${Array.from(frame.data).map((b: number) => b.toString(16).padStart(2, '0')).join(' ')}`);

      if ((frame.mid === 0x01 || frame.mid === (MID.STOP_INVENTORY & 0xFF)) && frame.data[0] === 0x00) {
        this.log('info', 'Reader successfully initialized');
        return true;
      }

      this.log('warn', 'Invalid STOP response');
      return false;
    } catch (error) {
      this.log('error', `Initialization error: ${(error as Error).message}`);
      return false;
    }
  }

  /**
   * Query reader information
   */
  async queryReaderInformation(): Promise<ReaderInformation> {
    try {
      const frame = NRNUtils.buildFrame(MID.QUERY_INFO, new Uint8Array(0));
      await this.send(frame);

      await this.delay(100);
      const raw = await this.receive();

      if (!raw) {
        this.log('warn', 'No response received for reader information query');
        return {};
      }

      const frameData = NRNUtils.parseFrame(raw);
      if (frameData.mid !== 0x00 || frameData.category !== 0x01) {
        this.log('warn', 'Unexpected MID or Category in reader information response');
        return {};
      }

      return this.parseQueryInfoData(frameData.data);
    } catch (error) {
      this.log('error', `Reader information query error: ${(error as Error).message}`);
      return {};
    }
  }

  /**
   * Parse query info data
   */
  private parseQueryInfoData(data: Uint8Array): ReaderInformation {
    const result: ReaderInformation = {};
    let offset = 0;

    try {
      // Serial Number
      if (offset + 2 <= data.length) {
        const snLength = data[offset + 1];
        const serial = new TextDecoder().decode(data.slice(offset + 2, offset + 2 + snLength));
        result.serial_number = serial.trim();
        offset += 2 + snLength;
      }

      // Power-on time
      if (offset + 4 <= data.length) {
        result.power_on_time_sec = (data[offset] << 24) | (data[offset + 1] << 16) |
          (data[offset + 2] << 8) | data[offset + 3];
        offset += 4;
      }

      // Baseband compile time
      if (offset + 2 <= data.length) {
        const bbLen = data[offset + 1];
        const baseband = new TextDecoder().decode(data.slice(offset + 2, offset + 2 + bbLen));
        result.baseband_compile_time = baseband.trim();
        offset += 2 + bbLen;
      }

      // Optional tags
      while (offset + 2 <= data.length) {
        const tag = data[offset];
        const length = data[offset + 1];
        const value = data.slice(offset + 2, offset + 2 + length);
        offset += 2 + length;

        if (tag === 0x01 && value.length === 4) {
          const v = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | value[3];
          result.app_version = `V${(v >> 24) & 0xFF}.${(v >> 16) & 0xFF}.${(v >> 8) & 0xFF}.${v & 0xFF}`;
        } else if (tag === 0x02) {
          result.os_version = new TextDecoder().decode(value).trim();
        } else if (tag === 0x03) {
          result.app_compile_time = new TextDecoder().decode(value).trim();
        }
      }
    } catch (error) {
      result.error = `Parsing exception: ${(error as Error).message}`;
    }

    return result;
  }

  /**
   * Start inventory with callback
   */
  async startInventory(antennaMask: number[] = [1], callback: TagCallback | null = null): Promise<boolean> {
    try {
      await this.stopInventory();
      this.isInventoryRunning = true;
      this.inventoryCallback = callback;

      const mask = this.buildAntennaMask(antennaMask);
      this.log('info', `Starting inventory with antenna mask: 0x${mask.toString(16).padStart(8, '0')}`);

      const payload = this.buildEPCReadPayload(mask, true);
      const frame = NRNUtils.buildFrame(MID.READ_EPC_TAG, payload, this.rs485);

      await this.send(frame);

      // Start receiving loop
      this.receiveInventoryLoop();

      return true;
    } catch (error) {
      this.log('error', `Inventory start error: ${(error as Error).message}`);
      return false;
    }
  }

  /**
   * Stop inventory
   */
  async stopInventory(): Promise<boolean> {
    try {
      this.isInventoryRunning = false;
      this.inventoryCallback = null;

      const stopFrame = NRNUtils.buildFrame(MID.STOP_INVENTORY, new Uint8Array(0));
      this.log('debug', `Sending stop frame: ${Array.from(stopFrame).map((b: number) => b.toString(16).padStart(2, '0')).join(' ')}`);

      await this.send(stopFrame);

      // Wait for confirmation
      for (let attempt = 0; attempt < 10; attempt++) {
        await this.delay(200);

        try {
          const raw = await this.receive();
          const frames = NRNUtils.extractValidFrames(raw, this.rs485);

          for (const frame of frames) {
            try {
              const resp = NRNUtils.parseFrame(frame);
              const mid = resp.mid;
              const data = resp.data;

              if (mid === MID.STOP_OPERATION) {
                const result = data.length > 0 ? data[0] : -1;
                if (result === 0x00) {
                  this.log('info', 'Reader responded: STOP successful, now IDLE');
                  return true;
                } else {
                  this.log('warn', `Reader responded: STOP error code 0x${result.toString(16)}`);
                  return false;
                }
              } else if (this.isReadEndMID(mid)) {
                const reason = data.length > 0 ? data[0] : -1;
                if (reason === 1) {
                  this.log('info', 'Read end notification: stopped by STOP command');
                  return true;
                } else {
                  this.log('warn', `Read ended with reason code ${reason}, not STOP command`);
                }
              }
            } catch (error) {
              this.log('debug', `Frame parse error: ${(error as Error).message}`);
            }
          }
        } catch (error) {
          this.log('debug', `UART receive error on attempt ${attempt + 1}: ${(error as Error).message}`);
        }
      }

      this.log('warn', 'STOP failed: no valid response after 10 attempts');
      return false;
    } catch (error) {
      this.log('error', `Stop inventory error: ${(error as Error).message}`);
      return false;
    }
  }

  /**
   * Build antenna mask from antenna IDs
   */
  private buildAntennaMask(antennaIds: number[]): number {
    let mask = 0;
    for (const aid of antennaIds) {
      if (aid < 1 || aid > 32) {
        throw new ConfigurationError(`Antenna ID ${aid} out of valid range (1-32)`);
      }
      mask |= (1 << (aid - 1));
    }
    return mask;
  }

  /**
   * Build EPC read payload with optional TID reading
   * Aligned with protocols.ts generateFastSwitchData
   */
  private buildEPCReadPayload(antennaMask: number, continuous: boolean = true, includeTid: boolean = false): Uint8Array {
    if (!antennaMask) {
      antennaMask = 0x00000001;
    }

    const data: number[] = [
      (antennaMask >> 24) & 0xFF,
      (antennaMask >> 16) & 0xFF,
      (antennaMask >> 8) & 0xFF,
      antennaMask & 0xFF,
      continuous ? 0x01 : 0x00,  // M: Constant reading (Continuous)
    ];

    // Only add TID reading parameters if includeTid is true
    if (includeTid) {
      data.push(0x02);  // PID 0x02: TID reading parameters
      data.push(0x00);  // Byte 0: TID reading mode (0: self-adaptable)
      data.push(0x00);  // Byte 1: Word length (0x00 = Auto / Max)
    }

    return new Uint8Array(data);
  }

  /**
   * Parse EPC tag data with optional PIDs (RSSI, TID, phase, frequency)
   * Aligned with protocols.ts NATION protocol parsing
   */
  private parseEPC(data: Uint8Array): TagData {
    try {
      // 1. Read EPC (Mandatory)
      const epcLen = (data[0] << 8) | data[1];
      const epc = Array.from(data.slice(2, 2 + epcLen))
        .map((b: number) => b.toString(16).padStart(2, '0'))
        .join('')
        .toUpperCase();

      // 2. Read PC (Mandatory - 2 bytes)
      const pc = Array.from(data.slice(2 + epcLen, 2 + epcLen + 2))
        .map((b: number) => b.toString(16).padStart(2, '0'))
        .join('')
        .toUpperCase();

      // 3. Read Antenna ID (Mandatory - 1 byte)
      const antennaId = data[2 + epcLen + 2];

      // 4. Parse optional PIDs starting after Antenna
      let rssi: number | null = null;
      let tid: string | undefined = undefined;
      let phase: number | undefined = undefined;
      let frequency: number | undefined = undefined;
      let cursor = 2 + epcLen + 3;

      while (cursor < data.length) {
        const pid = data[cursor];
        cursor++;

        if (pid === 0x01) { // RSSI (1 byte)
          if (cursor < data.length) {
            const rssiRaw = data[cursor];
            // Convert to dBm using formula from protocols.ts
            rssi = -100 + Math.round((rssiRaw * 70) / 255);
            cursor += 1;
          } else break;
        }
        else if (pid === 0x02) { // Reading Result (1 byte)
          if (cursor < data.length) {
            cursor += 1;
          } else break;
        }
        else if (pid === 0x03) { // TID DATA (Variable length)
          if (cursor + 1 < data.length) {
            const tidByteLen = (data[cursor] << 8) | data[cursor + 1];
            cursor += 2;
            if (cursor + tidByteLen <= data.length) {
              tid = Array.from(data.slice(cursor, cursor + tidByteLen))
                .map((b: number) => b.toString(16).padStart(2, '0'))
                .join('')
                .toUpperCase();
              cursor += tidByteLen;
            } else break;
          } else break;
        }
        else if (pid === 0x04 || pid === 0x05) { // Tag data area or reserve area (Variable length)
          if (cursor + 1 < data.length) {
            const byteLen = (data[cursor] << 8) | data[cursor + 1];
            cursor += 2 + byteLen;
          } else break;
        }
        else if (pid === 0x06) { // Sub-antenna No (1 byte)
          cursor += 1;
        }
        else if (pid === 0x07) { // UTC reading time (8 bytes)
          cursor += 8;
        }
        else if (pid === 0x08) { // Current frequency (4 bytes, KHz)
          if (cursor + 3 < data.length) {
            const freqKHz = (data[cursor] << 24) | (data[cursor + 1] << 16) | (data[cursor + 2] << 8) | data[cursor + 3];
            frequency = freqKHz / 1000.0;
            cursor += 4;
          } else break;
        }
        else if (pid === 0x09) { // Current tag phase (1 byte, 0~128)
          if (cursor < data.length) {
            const phaseRaw = data[cursor];
            phase = (phaseRaw / 128.0) * 2 * Math.PI;
            cursor += 1;
          } else break;
        }
        else {
          // Unknown PID - skip and continue
          continue;
        }
      }

      return {
        epc,
        pc,
        antenna_id: antennaId,
        rssi,
        tid,
        phase,
        frequency
      };
    } catch (error) {
      return {
        epc: '',
        pc: '',
        antenna_id: 0,
        rssi: null,
        error: `Parse error: ${(error as Error).message}`
      };
    }
  }

  /**
   * Check if MID is a read end MID
   */
  private isReadEndMID(mid: number): boolean {
    return [0x01, 0x21, 0x31].includes(mid);
  }

  /**
   * Receive inventory loop
   */
  private async receiveInventoryLoop(): Promise<void> {
    let buffer = new Uint8Array(0);

    while (this.isInventoryRunning) {
      try {
        const raw = await this.receive();
        if (!raw) {
          await this.delay(10);
          continue;
        }

        buffer = this.concatUint8Arrays(buffer, raw);
        const frames = NRNUtils.extractValidFrames(buffer, this.rs485);

        // Remove processed bytes from buffer
        if (frames.length > 0) {
          const lastFrame = frames[frames.length - 1];
          const lastFrameIndex = this.findUint8ArrayIndex(buffer, lastFrame);
          if (lastFrameIndex !== -1) {
            buffer = buffer.slice(lastFrameIndex + lastFrame.length);
          }
        }

        for (const frame of frames) {
          try {
            const parsed = NRNUtils.parseFrame(frame);
            const cat = parsed.category;
            const mid = parsed.mid;

            if (cat === 0x10 || mid === 0x00) {
              // EPC tag
              const tag = this.parseEPC(parsed.data);
              if (!tag.error && this.inventoryCallback) {
                this.inventoryCallback(tag);
              }
            } else if (this.isReadEndMID(mid)) {
              const reason = parsed.data.length > 0 ? parsed.data[0] : null;
              this.log('info', `Inventory ended. Reason: ${reason}`);
              this.isInventoryRunning = false;
              return;
            }
          } catch (error) {
            this.log('debug', `Frame parse error: ${(error as Error).message}`);
          }
        }
      } catch (error) {
        this.log('debug', `Inventory loop error: ${(error as Error).message}`);
        await this.delay(10);
      }
    }
  }

  /**
   * Concatenate two Uint8Arrays
   */
  private concatUint8Arrays(a: Uint8Array, b: Uint8Array): Uint8Array {
    const result = new Uint8Array(a.length + b.length);
    result.set(a);
    result.set(b, a.length);
    return result;
  }

  /**
   * Find index of Uint8Array in another Uint8Array
   */
  private findUint8ArrayIndex(haystack: Uint8Array, needle: Uint8Array): number {
    for (let i = 0; i <= haystack.length - needle.length; i++) {
      let found = true;
      for (let j = 0; j < needle.length; j++) {
        if (haystack[i + j] !== needle[j]) {
          found = false;
          break;
        }
      }
      if (found) return i;
    }
    return -1;
  }

  /**
   * Delay execution
   */
  private delay(ms: number): Promise<void> {
    return new Promise<void>((resolve) => setTimeout(resolve, ms));
  }
}

// === Factory Function ===
export function createNRNReader(options: NRNWebSerialOptions = {}): NRNWebSerial {
  return new NRNWebSerial(options);
}

// === Default Export ===
export default NRNWebSerial;
