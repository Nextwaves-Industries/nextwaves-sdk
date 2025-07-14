class MessageTran {
    constructor() {
        this.packetType = 0xA0; // Default packet type (160)
        this.dataLen = 0;
        this.readId = 0;
        this.cmd = 0;
        this.data = [];
        this.check = 0;
        this.tranData = [];
    }

    static checksum(buffer, startPos, length) {
        let sum = 0;
        for (let i = startPos; i < startPos + length; i++) {
            sum += buffer[i];
        }
        return (~sum + 1) & 0xff;
    }

    createMessage(readId, cmd, data = []) {
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
    constructor() {
        this.port = null;
        this.reader = null;
        this.writer = null;
        this.isConnected = false;
        this.isReading = false;
        this.buffer = [];
        this.tags = new Map(); // Store unique tags
        this.onDataReceived = null;
        this.onTagsUpdated = null;
        this.onLogMessage = null;
    }

    async connect() {
        try {
            // Request port access
            this.port = await navigator.serial.requestPort();
            
            // Open the port with the same settings as Python
            await this.port.open({ baudRate: 115200 });
            
            this.isConnected = true;
            this.log('Connected to reader');
            
            // Start reading data
            this.startReading();
            
            return true;
        } catch (error) {
            this.log(`Connection failed: ${error.message}`);
            return false;
        }
    }

    async disconnect() {
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
        }
    }

    async startReading() {
        if (!this.isConnected) return;

        this.isReading = true;
        const textDecoder = new TextDecoderStream();
        const readableStreamClosed = this.port.readable.pipeTo(textDecoder.writable);
        this.reader = textDecoder.readable.getReader();

        try {
            while (this.isReading) {
                const { value, done } = await this.reader.read();
                if (done) {
                    break;
                }
                
                // Convert text to bytes
                const bytes = new Uint8Array(value.length);
                for (let i = 0; i < value.length; i++) {
                    bytes[i] = value.charCodeAt(i);
                }
                
                this.processReceivedData(Array.from(bytes));
            }
        } catch (error) {
            this.log(`Reading error: ${error.message}`);
        } finally {
            this.reader.releaseLock();
        }
    }

    async sendMessage(message) {
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
            this.log(`Failed to send message: ${error.message}`);
            return false;
        }
    }

    processReceivedData(data) {
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

    processFrame(frame) {
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

    parseInventoryData(frame) {
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
            epcEnd = frame.length - 2;
        }
        
        const epcBytes = frame.slice(epcStart, epcEnd);
        const epc = this.arrayToHex(epcBytes).toUpperCase();
        
        if (epc) {
            const rssiByte = frame[frame.length - 2];
            const rssi = -100 + rssiByte;
            const frequency = 865.0 + (chIdx * 0.5);
            
            const tag = {
                epc: epc,
                antenna: antenna,
                frequency: frequency,
                rssi: rssi,
                timestamp: new Date().toISOString()
            };
            
            // Store tag (update if exists)
            this.tags.set(epc, tag);
            
            this.log(`Tag detected: ${epc} (Antenna: ${antenna}, RSSI: ${rssi}dBm, Freq: ${frequency}MHz)`);
            
            if (this.onTagsUpdated) {
                this.onTagsUpdated(Array.from(this.tags.values()));
            }
        }
    }

    async setPower(p1 = 30, p2 = 30, p3 = 30, p4 = 10) {
        if (!this.validatePower(p1, p2, p3, p4)) {
            this.log('Power values must be between 0-33 dBm');
            return false;
        }
        
        const message = new MessageTran();
        const cmd = message.createMessage(0x76, 0x07, [p1, p2, p3, p4]);
        return await this.sendMessage(cmd);
    }

    async startInventory(readId = 0xFF) {
        const message = new MessageTran();
        const cmd = message.createMessage(readId, 0x8A, [0x00, 0x01, 0x01, 0x01, 0x02, 0x01, 0x03, 0x01, 0x01, 0x01]);
        return await this.sendMessage(cmd);
    }

    async stopInventory(readId = 0xFF) {
        const message = new MessageTran();
        const cmd = message.createMessage(readId, 0x8B);
        return await this.sendMessage(cmd);
    }

    async getInfo() {
        const message = new MessageTran();
        const cmd = message.createMessage(0x76, 0x01);
        return await this.sendMessage(cmd);
    }

    validatePower(...powers) {
        return powers.every(p => p >= 0 && p <= 33);
    }

    arrayToHex(array) {
        return Array.from(array, byte => byte.toString(16).padStart(2, '0')).join(' ');
    }

    log(message) {
        const timestamp = new Date().toLocaleTimeString();
        const logMessage = `[${timestamp}] ${message}`;
        console.log(logMessage);
        
        if (this.onLogMessage) {
            this.onLogMessage(logMessage);
        }
    }

    clearTags() {
        this.tags.clear();
        if (this.onTagsUpdated) {
            this.onTagsUpdated([]);
        }
    }
}

// UI Controller
class ReaderUI {
    constructor() {
        this.reader = new NextWavesReader();
        this.setupEventListeners();
        this.setupCallbacks();
    }

    setupEventListeners() {
        // Connection buttons
        document.getElementById('connectBtn').addEventListener('click', () => this.connect());
        document.getElementById('disconnectBtn').addEventListener('click', () => this.disconnect());

        // Control buttons
        document.getElementById('setPowerBtn').addEventListener('click', () => this.setPower());
        document.getElementById('startInventoryBtn').addEventListener('click', () => this.startInventory());
        document.getElementById('stopInventoryBtn').addEventListener('click', () => this.stopInventory());
        document.getElementById('getInfoBtn').addEventListener('click', () => this.getInfo());
    }

    setupCallbacks() {
        this.reader.onLogMessage = (message) => {
            this.addLogMessage(message);
        };

        this.reader.onTagsUpdated = (tags) => {
            this.updateTagsDisplay(tags);
        };
    }

    async connect() {
        this.updateConnectionStatus('connecting', 'Connecting...');
        
        const success = await this.reader.connect();
        
        if (success) {
            this.updateConnectionStatus('connected', 'Connected');
            this.updateButtonStates(true);
            document.getElementById('controlsSection').classList.remove('hidden');
        } else {
            this.updateConnectionStatus('disconnected', 'Connection failed');
        }
    }

    async disconnect() {
        await this.reader.disconnect();
        this.updateConnectionStatus('disconnected', 'Disconnected');
        this.updateButtonStates(false);
        document.getElementById('controlsSection').classList.add('hidden');
    }

    async setPower() {
        const p1 = parseInt(document.getElementById('power1').value);
        const p2 = parseInt(document.getElementById('power2').value);
        const p3 = parseInt(document.getElementById('power3').value);
        const p4 = parseInt(document.getElementById('power4').value);
        
        await this.reader.setPower(p1, p2, p3, p4);
    }

    async startInventory() {
        await this.reader.startInventory();
    }

    async stopInventory() {
        await this.reader.stopInventory();
    }

    async getInfo() {
        await this.reader.getInfo();
    }

    updateConnectionStatus(status, message) {
        const statusElement = document.getElementById('connectionStatus');
        statusElement.className = `status ${status}`;
        statusElement.textContent = `Status: ${message}`;
    }

    updateButtonStates(connected) {
        document.getElementById('connectBtn').disabled = connected;
        document.getElementById('disconnectBtn').disabled = !connected;
        document.getElementById('setPowerBtn').disabled = !connected;
        document.getElementById('startInventoryBtn').disabled = !connected;
        document.getElementById('stopInventoryBtn').disabled = !connected;
        document.getElementById('getInfoBtn').disabled = !connected;
    }

    updateTagsDisplay(tags) {
        const container = document.getElementById('tagsContainer');
        
        if (tags.length === 0) {
            container.innerHTML = '<div style="padding: 20px; text-align: center; color: #6c757d;">No tags detected yet. Connect and start inventory to begin reading.</div>';
            return;
        }
        
        container.innerHTML = tags.map(tag => `
            <div class="tag-item">
                <div class="tag-info">
                    <div class="tag-epc">${tag.epc}</div>
                    <div class="tag-details">
                        Antenna: ${tag.antenna} | 
                        Frequency: ${tag.frequency.toFixed(1)} MHz | 
                        Time: ${new Date(tag.timestamp).toLocaleTimeString()}
                    </div>
                </div>
                <div class="tag-rssi">${tag.rssi} dBm</div>
            </div>
        `).join('');
    }

    addLogMessage(message) {
        const container = document.getElementById('logContainer');
        container.textContent += message + '\n';
        container.scrollTop = container.scrollHeight;
    }
}

// Initialize the UI when the page loads
document.addEventListener('DOMContentLoaded', () => {
    new ReaderUI();
}); 