// Type declarations for WebSerial API
declare global {
  interface Navigator {
    serial: {
      requestPort(): Promise<SerialPort>;
    };
  }
  
  interface SerialPort {
    open(options: { baudRate: number }): Promise<void>;
    close(): Promise<void>;
    readable: ReadableStream<Uint8Array>;
    writable: WritableStream<Uint8Array>;
  }
}

// Types for the VMR64 reader
interface Tag {
  epc: string;
  antenna: number;
  frequency: number;
  rssi: number;
  timestamp: string;
}

interface MessageTranData {
  packetType: number;
  dataLen: number;
  readId: number;
  cmd: number;
  data: number[];
  check: number;
  tranData: number[];
}

interface ReaderCallbacks {
  onDataReceived?: (data: number[]) => void;
  onTagsUpdated?: (tags: Tag[]) => void;
  onLogMessage?: (message: string) => void;
  onConnectionChange?: (connected: boolean) => void;
}

class MessageTran {
  packetType: number = 0xA0;
  dataLen: number = 0;
  readId: number = 0;
  cmd: number = 0;
  data: number[] = [];
  check: number = 0;
  tranData: number[] = [];

  static checksum(buffer: number[], startPos: number, length: number): number {
    let sum = 0;
    for (let i = startPos; i < startPos + length; i++) {
      sum += buffer[i];
    }
    return (~sum + 1) & 0xff;
  }

  createMessage(readId: number, cmd: number, data: number[] = []): number[] {
    this.readId = readId;
    this.cmd = cmd;
    this.data = data;
    this.dataLen = this.data.length + 3;
    
    // Create base message
    this.tranData = [this.packetType, this.dataLen, this.readId, this.cmd];
    
    // Add data if present
    if (this.data.length > 0) {
      this.tranData.push(...this.data);
    }
    
    // Calculate and add checksum
    this.check = MessageTran.checksum(this.tranData, 0, this.tranData.length);
    this.tranData.push(this.check);
    
    return this.tranData;
  }
}

class NextWavesReader {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private isConnected: boolean = false;
  private isReading: boolean = false;
  private buffer: number[] = [];
  private tags: Map<string, Tag> = new Map();
  private callbacks: ReaderCallbacks;

  constructor(callbacks: ReaderCallbacks = {}) {
    this.callbacks = callbacks;
  }

  async connect(): Promise<boolean> {
    try {
      // Request port access
      this.port = await navigator.serial.requestPort();
      
      // Open the port with the same settings as Python
      await this.port.open({ baudRate: 115200 });
      
      this.isConnected = true;
      this.log('Connected to reader');
      this.callbacks.onConnectionChange?.(true);
      
      // Start reading data
      this.startReading();
      
      return true;
    } catch (error) {
      this.log(`Connection failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
      return false;
    }
  }

  async disconnect(): Promise<void> {
    if (this.isConnected) {
      this.isReading = false;
      if (this.reader) {
        await this.reader.cancel();
      }
      if (this.port) {
        await this.port.close();
      }
      this.isConnected = false;
      this.log('Disconnected from reader');
      this.callbacks.onConnectionChange?.(false);
    }
  }

  private async startReading(): Promise<void> {
    if (!this.isConnected || !this.port) return;

    this.isReading = true;
    this.reader = this.port.readable.getReader();

    try {
      while (this.isReading) {
        const { value, done } = await this.reader.read();
        if (done) {
          break;
        }
        
        // Process received bytes directly
        this.processReceivedData(Array.from(value));
      }
    } catch (error) {
      this.log(`Reading error: ${error instanceof Error ? error.message : 'Unknown error'}`);
    } finally {
      this.reader?.releaseLock();
    }
  }

  async sendMessage(message: number[]): Promise<boolean> {
    if (!this.isConnected || !this.port) {
      this.log('Reader not connected');
      return false;
    }

    try {
      const writer = this.port.writable.getWriter();
      const data = new Uint8Array(message);
      await writer.write(data);
      writer.releaseLock();
      
      this.log(`Sent: ${this.arrayToHex(message)}`);
      return true;
    } catch (error) {
      this.log(`Failed to send message: ${error instanceof Error ? error.message : 'Unknown error'}`);
      return false;
    }
  }

  private processReceivedData(data: number[]): void {
    this.buffer.push(...data);
    this.log(`Received: ${this.arrayToHex(data)}`);
    
    // Process complete messages
    let i = 0;
    while (i < this.buffer.length) {
      // Look for packet start (0xA0)
      if (this.buffer[i] !== 0xA0) {
        i++;
        continue;
      }
      
      // Check if we have enough data for length
      if (i + 1 >= this.buffer.length) break;
      
      const length = this.buffer[i + 1];
      const frameLen = length + 2;
      
      // Check if we have complete frame
      if (i + frameLen > this.buffer.length) break;
      
      const frame = this.buffer.slice(i, i + frameLen);
      
      // Process the frame
      this.processFrame(frame);
      
      // Remove processed data from buffer
      this.buffer.splice(0, i + frameLen);
      i = 0;
    }
  }

  private processFrame(frame: number[]): void {
    if (frame.length < 5) return;
    
    const cmd = frame[3];
    
    switch (cmd) {
      case 0x8A: // Inventory response
        this.parseInventoryData(frame);
        break;
      case 0x01: // Info response
        this.log('Reader info received');
        break;
      case 0x07: // Power set response
        this.log('Power settings applied');
        break;
      case 0x8B: // Stop inventory response
        this.log('Inventory stopped');
        break;
      default:
        this.log(`Unknown command: 0x${cmd.toString(16).toUpperCase()}`);
    }
  }

  private parseInventoryData(frame: number[]): void {
    if (frame.length < 7) return;
    
    const freqAnt = frame[4];
    const antenna = (freqAnt & 0x03) + 1;
    const chIdx = freqAnt >> 2;
    
    const pcLow = frame[5];
    const epcWords = (pcLow >> 3) & 0x1F;
    const epcLen = epcWords * 2;
    
    const epcStart = 7;
    const epcEnd = epcStart + epcLen;
    
    if (epcEnd > frame.length - 2) {
      return;
    }
    
    const epcBytes = frame.slice(epcStart, epcEnd);
    const epc = this.arrayToHex(epcBytes).toUpperCase();
    
    if (epc) {
      const rssiByte = frame[frame.length - 2];
      const rssi = -100 + rssiByte;
      const frequency = 865.0 + (chIdx * 0.5);
      
      const tag: Tag = {
        epc: epc,
        antenna: antenna,
        frequency: frequency,
        rssi: rssi,
        timestamp: new Date().toISOString()
      };
      
      // Store tag (update if exists)
      this.tags.set(epc, tag);
      
      this.log(`Tag detected: ${epc} (Antenna: ${antenna}, RSSI: ${rssi}dBm, Freq: ${frequency}MHz)`);
      
      if (this.callbacks.onTagsUpdated) {
        this.callbacks.onTagsUpdated(Array.from(this.tags.values()));
      }
    }
  }

  async setPower(p1: number = 30, p2: number = 30, p3: number = 30, p4: number = 10): Promise<boolean> {
    if (!this.validatePower(p1, p2, p3, p4)) {
      this.log('Power values must be between 0-33 dBm');
      return false;
    }
    
    const message = new MessageTran();
    const cmd = message.createMessage(0x76, 0x07, [p1, p2, p3, p4]);
    return await this.sendMessage(cmd);
  }

  async startInventory(readId: number = 0xFF): Promise<boolean> {
    const message = new MessageTran();
    const cmd = message.createMessage(readId, 0x8A, [0x00, 0x01, 0x01, 0x01, 0x02, 0x01, 0x03, 0x01, 0x01, 0x01]);
    return await this.sendMessage(cmd);
  }

  async stopInventory(readId: number = 0xFF): Promise<boolean> {
    const message = new MessageTran();
    const cmd = message.createMessage(readId, 0x8B);
    return await this.sendMessage(cmd);
  }

  async getInfo(): Promise<boolean> {
    const message = new MessageTran();
    const cmd = message.createMessage(0x76, 0x01);
    return await this.sendMessage(cmd);
  }

  private validatePower(...powers: number[]): boolean {
    return powers.every(p => p >= 0 && p <= 33);
  }

  private arrayToHex(array: number[]): string {
    return Array.from(array, byte => byte.toString(16).padStart(2, '0')).join(' ');
  }

  private log(message: string): void {
    const timestamp = new Date().toLocaleTimeString();
    const logMessage = `[${timestamp}] ${message}`;
    console.log(logMessage);
    
    if (this.callbacks.onLogMessage) {
      this.callbacks.onLogMessage(logMessage);
    }
  }

  clearTags(): void {
    this.tags.clear();
    if (this.callbacks.onTagsUpdated) {
      this.callbacks.onTagsUpdated([]);
    }
  }

  getTags(): Tag[] {
    return Array.from(this.tags.values());
  }

  isReaderConnected(): boolean {
    return this.isConnected;
  }
}

// React component that uses the VMR64 reader
interface VMR64ReaderProps {
  onTagsUpdate?: (tags: Tag[]) => void;
  onLogMessage?: (message: string) => void;
  onConnectionChange?: (connected: boolean) => void;
}

interface VMR64ReaderState {
  isConnected: boolean;
  tags: Tag[];
  logMessages: string[];
}

class VMR64Reader {
  private reader: NextWavesReader;
  private props: VMR64ReaderProps;

  constructor(props: VMR64ReaderProps) {
    this.props = props;

    this.reader = new NextWavesReader({
      onTagsUpdated: (tags) => {
        this.props.onTagsUpdate?.(tags);
      },
      onLogMessage: (message) => {
        this.props.onLogMessage?.(message);
      },
      onConnectionChange: (connected) => {
        this.props.onConnectionChange?.(connected);
      }
    });
  }

  async connect(): Promise<void> {
    await this.reader.connect();
  }

  async disconnect(): Promise<void> {
    await this.reader.disconnect();
  }

  async setPower(p1: number, p2: number, p3: number, p4: number): Promise<void> {
    await this.reader.setPower(p1, p2, p3, p4);
  }

  async startInventory(): Promise<void> {
    await this.reader.startInventory();
  }

  async stopInventory(): Promise<void> {
    await this.reader.stopInventory();
  }

  async getInfo(): Promise<void> {
    await this.reader.getInfo();
  }

  clearTags(): void {
    this.reader.clearTags();
  }

  getTags(): Tag[] {
    return this.reader.getTags();
  }

  isConnected(): boolean {
    return this.reader.isReaderConnected();
  }
}

export default VMR64Reader;
export { NextWavesReader, MessageTran, Tag, ReaderCallbacks };
