// Package nrn provides a comprehensive SDK for communicating with NRN RFID readers.
//
// Example usage:
//
//	reader, err := nrn.NewNRNReader("/dev/ttyUSB0", 115200)
//	if err != nil {
//	    log.Fatal(err)
//	}
//	defer reader.Close()
//
//	if err := reader.ConnectAndInitialize(); err != nil {
//	    log.Fatal(err)
//	}
//
//	err = reader.StartInventory(0x01, func(tag nrn.TagData) {
//	    fmt.Printf("EPC: %s, RSSI: %d dBm\n", tag.EPC, *tag.RSSI)
//	})
package nrn

import (
	"encoding/binary"
	"encoding/hex"
	"fmt"
	"log"
	"math"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"go.bug.st/serial"
)

// === Constants ===

const (
	FrameHeader     byte   = 0x5A
	CRC16CCITTInit  uint16 = 0x0000
	CRC16CCITTPoly  uint16 = 0x8005
	DefaultBaudrate        = 115200
	DefaultTimeout         = 500 * time.Millisecond
)

// === Message IDs ===

type MID uint16

const (
	// Reader Configuration
	MIDQueryInfo          MID = 0x0100
	MIDConfirmConnection  MID = 0x12

	// RFID Inventory
	MIDReadEpcTag      MID = 0x0210
	MIDPhaseInventory  MID = 0x0214
	MIDStopInventory   MID = 0x02FF
	MIDWriteEpcTag     MID = 0x0211

	// Error Handling
	MIDErrorNotification MID = 0x00

	// RFID Baseband
	MIDConfigBaseband MID = 0x020B
	MIDQueryBaseband  MID = 0x020C

	// Power Control
	MIDConfigureReaderPower   MID = 0x0201
	MIDQueryReaderPower       MID = 0x0202
	MIDReaderPowerCalibration MID = 0x0103
	MIDQueryPowerCalibration  MID = 0x0104

	// Filter Settings
	MIDSetFilterSettings   MID = 0x0209
	MIDQueryFilterSettings MID = 0x020A

	// RF Band & Frequency
	MIDSetRfBand            MID = 0x0203
	MIDQueryRfBand          MID = 0x0204
	MIDSetWorkingFrequency  MID = 0x0205
	MIDQueryWorkingFrequency MID = 0x0206

	// RFID Ability
	MIDQueryRfidAbility MID = 0x1000

	// Buzzer Control
	MIDBuzzerSwitch MID = 0x011E

	// GPIO Commands
	MIDConfigureGpo       MID = 0x0109
	MIDQueryGpi           MID = 0x010A
	MIDConfigureGpiTrigger MID = 0x010B
	MIDQueryGpiTrigger    MID = 0x010C
)

// === Beeper Modes ===

type BeeperMode uint8

const (
	BeeperQuiet            BeeperMode = 0x00
	BeeperAfterInventory   BeeperMode = 0x01
	BeeperAfterTag         BeeperMode = 0x02
)

// === RF Profiles ===

type RFProfile struct {
	ID          int
	Name        string
	Description string
}

var RFProfiles = []RFProfile{
	{0, "Profile 0", "Default baseband profile"},
	{1, "Profile 1", "High performance profile"},
	{2, "Profile 2", "Dense tag profile"},
}

// === Data Structures ===

// TagData represents parsed tag information from inventory response
type TagData struct {
	EPC       string
	PC        string
	AntennaID uint8
	RSSI      *int32
	TID       *string
	Phase     *float64
	Frequency *float64
}

// ReaderInfo contains reader device information
type ReaderInfo struct {
	SerialNumber        string
	PowerOnTimeSec      uint32
	BasebandCompileTime string
	AppVersion          string
	OSVersion           string
	AppCompileTime      string
}

// RFIDAbility contains reader capabilities
type RFIDAbility struct {
	MinPower     int32
	MaxPower     int32
	AntennaCount uint8
	Frequencies  []float64
}

// ParsedFrame represents a parsed protocol frame
type ParsedFrame struct {
	Valid      bool
	ProtoType  uint8
	ProtoVer   uint8
	RS485      bool
	Notify     bool
	Category   uint8
	MID        uint8
	DataLength uint16
	Data       []byte
	CRC        uint16
}

// TagCallback is the function signature for tag callbacks
type TagCallback func(tag TagData)

// === Utility Functions ===

// CRC16CCITT calculates CRC16-CCITT checksum
func CRC16CCITT(data []byte) uint16 {
	crc := CRC16CCITTInit
	for _, b := range data {
		crc ^= uint16(b) << 8
		for i := 0; i < 8; i++ {
			if (crc & 0x8000) != 0 {
				crc = (crc << 1) ^ CRC16CCITTPoly
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}

// CalculateRSSI converts raw RSSI byte value to dBm
// Formula: -100 + round((rssiRaw * 70) / 255)
func CalculateRSSI(rssiRaw uint8) int32 {
	return -100 + int32((int(rssiRaw)*70+127)/255)
}

// CalculateFrequency converts channel index to MHz
// Formula: 920.0 + chIdx * 0.5
func CalculateFrequency(chIdx uint8) float64 {
	return 920.0 + float64(chIdx)*0.5
}

// BytesToHex converts bytes to uppercase hex string
func BytesToHex(data []byte) string {
	return strings.ToUpper(hex.EncodeToString(data))
}

// BuildAntennaMask creates antenna mask from list of antenna IDs
func BuildAntennaMask(antennas []int) uint32 {
	var mask uint32
	for _, antID := range antennas {
		if antID >= 1 && antID <= 32 {
			mask |= 1 << (antID - 1)
		}
	}
	return mask
}

// === NRNReader ===

// NRNReader is the main SDK class for reader communication
type NRNReader struct {
	port        serial.Port
	portName    string
	baudrate    int
	timeout     time.Duration
	antennaMask uint32
	running     atomic.Bool
	mu          sync.Mutex
	callback    TagCallback
}

// NewNRNReader creates a new reader instance
func NewNRNReader(portName string, baudrate int) (*NRNReader, error) {
	mode := &serial.Mode{
		BaudRate: baudrate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(portName, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open port %s: %w", portName, err)
	}

	port.SetReadTimeout(DefaultTimeout)

	log.Printf("Opened port: %s @ %d baud", portName, baudrate)

	return &NRNReader{
		port:        port,
		portName:    portName,
		baudrate:    baudrate,
		timeout:     DefaultTimeout,
		antennaMask: 0x00000001,
	}, nil
}

// Close closes the serial connection
func (r *NRNReader) Close() error {
	r.StopInventory()
	if r.port != nil {
		return r.port.Close()
	}
	return nil
}

// ConnectAndInitialize initializes the reader
func (r *NRNReader) ConnectAndInitialize() error {
	frame := r.BuildFrame(MIDConfirmConnection, nil)
	_, err := r.port.Write(frame)
	if err != nil {
		return fmt.Errorf("failed to initialize reader: %w", err)
	}

	log.Println("Reader initialized")
	return nil
}

// BuildEPCReadPayload builds the payload for inventory command
func (r *NRNReader) BuildEPCReadPayload(antennaMask uint32, continuous bool, includeTID bool) []byte {
	if antennaMask == 0 {
		antennaMask = 0x00000001
	}

	data := []byte{
		byte((antennaMask >> 24) & 0xFF),
		byte((antennaMask >> 16) & 0xFF),
		byte((antennaMask >> 8) & 0xFF),
		byte(antennaMask & 0xFF),
	}

	if continuous {
		data = append(data, 0x01)
	} else {
		data = append(data, 0x00)
	}

	if includeTID {
		data = append(data, 0x02) // PID 0x02: TID reading parameters
		data = append(data, 0x00) // Byte 0: TID reading mode (0: self-adaptable)
		data = append(data, 0x00) // Byte 1: Word length (0x00 = Auto / Max)
	}

	return data
}

// BuildFrame builds a protocol frame
func (r *NRNReader) BuildFrame(mid MID, payload []byte) []byte {
	category := byte((uint16(mid) >> 8) & 0xFF)
	midByte := byte(uint16(mid) & 0xFF)

	frame := []byte{FrameHeader}

	// PCW: Proto Type, Proto Version, Flags, Category
	frame = append(frame, 0x00) // Proto Type
	frame = append(frame, 0x01) // Proto Version
	frame = append(frame, 0x00) // Flags
	frame = append(frame, category)

	// MID
	frame = append(frame, midByte)

	// Length
	payloadLen := uint16(len(payload))
	frame = append(frame, byte((payloadLen>>8)&0xFF))
	frame = append(frame, byte(payloadLen&0xFF))

	// Payload
	frame = append(frame, payload...)

	// CRC
	crc := CRC16CCITT(frame[1:])
	frame = append(frame, byte((crc>>8)&0xFF))
	frame = append(frame, byte(crc&0xFF))

	return frame
}

// ParseEPC parses tag data from inventory response
func (r *NRNReader) ParseEPC(data []byte) TagData {
	tag := TagData{}

	if len(data) < 5 {
		return tag
	}

	// 1. Read EPC (Mandatory)
	epcLen := int(binary.BigEndian.Uint16(data[0:2]))
	if len(data) < 2+epcLen+3 {
		return tag
	}

	tag.EPC = BytesToHex(data[2 : 2+epcLen])

	// 2. Read PC (Mandatory - 2 bytes)
	tag.PC = BytesToHex(data[2+epcLen : 2+epcLen+2])

	// 3. Read Antenna ID (Mandatory - 1 byte)
	tag.AntennaID = data[2+epcLen+2]

	// 4. Parse optional PIDs
	cursor := 2 + epcLen + 3

	for cursor < len(data) {
		pid := data[cursor]
		cursor++

		switch pid {
		case 0x01: // RSSI (1 byte)
			if cursor < len(data) {
				rssi := CalculateRSSI(data[cursor])
				tag.RSSI = &rssi
				cursor++
			} else {
				return tag
			}

		case 0x02: // Reading Result (1 byte)
			if cursor < len(data) {
				cursor++
			} else {
				return tag
			}

		case 0x03: // TID DATA (Variable length)
			if cursor+1 < len(data) {
				tidLen := int(binary.BigEndian.Uint16(data[cursor : cursor+2]))
				cursor += 2
				if cursor+tidLen <= len(data) {
					tid := BytesToHex(data[cursor : cursor+tidLen])
					tag.TID = &tid
					cursor += tidLen
				} else {
					return tag
				}
			} else {
				return tag
			}

		case 0x04, 0x05: // Tag/Reserve data
			if cursor+1 < len(data) {
				byteLen := int(binary.BigEndian.Uint16(data[cursor : cursor+2]))
				cursor += 2 + byteLen
			} else {
				return tag
			}

		case 0x06: // Sub-antenna
			cursor++

		case 0x07: // UTC time
			cursor += 8

		case 0x08: // Frequency (4 bytes, KHz)
			if cursor+3 < len(data) {
				freqKHz := binary.BigEndian.Uint32(data[cursor : cursor+4])
				freq := float64(freqKHz) / 1000.0
				tag.Frequency = &freq
				cursor += 4
			} else {
				return tag
			}

		case 0x09: // Phase (1 byte, 0~128)
			if cursor < len(data) {
				phaseRaw := data[cursor]
				phase := (float64(phaseRaw) / 128.0) * 2 * math.Pi
				tag.Phase = &phase
				cursor++
			} else {
				return tag
			}

		default:
			continue
		}
	}

	return tag
}

// StartInventory starts continuous inventory operation
func (r *NRNReader) StartInventory(antennaMask uint32, callback TagCallback) error {
	r.mu.Lock()
	defer r.mu.Unlock()

	r.antennaMask = antennaMask
	r.callback = callback
	r.running.Store(true)

	payload := r.BuildEPCReadPayload(antennaMask, true, false)
	frame := r.BuildFrame(MIDReadEpcTag, payload)

	_, err := r.port.Write(frame)
	if err != nil {
		return fmt.Errorf("failed to start inventory: %w", err)
	}

	log.Printf("Inventory started with mask: 0x%08X", antennaMask)
	return nil
}

// StopInventory stops the current inventory operation
func (r *NRNReader) StopInventory() error {
	r.running.Store(false)

	frame := r.BuildFrame(MIDStopInventory, nil)
	_, err := r.port.Write(frame)
	if err != nil {
		return fmt.Errorf("failed to stop inventory: %w", err)
	}

	log.Println("Inventory stopped")
	return nil
}

// ConfigurePower sets antenna power levels
func (r *NRNReader) ConfigurePower(powers map[int]int, persist bool) error {
	var payload []byte
	if persist {
		payload = append(payload, 0x01)
	} else {
		payload = append(payload, 0x00)
	}

	for antID, power := range powers {
		payload = append(payload, byte(antID))
		payload = append(payload, byte(power))
	}

	frame := r.BuildFrame(MIDConfigureReaderPower, payload)
	_, err := r.port.Write(frame)
	if err != nil {
		return fmt.Errorf("failed to configure power: %w", err)
	}

	log.Println("Power configured")
	return nil
}

// QueryPower returns current power settings
func (r *NRNReader) QueryPower() (map[int]int, error) {
	frame := r.BuildFrame(MIDQueryReaderPower, nil)
	response, err := r.sendAndReceive(frame)
	if err != nil {
		return nil, err
	}

	powers := make(map[int]int)
	// Response format: [persistence byte] + [ant_id, power] pairs
	if len(response) >= 1 {
		for i := 1; i+1 < len(response); i += 2 {
			antID := int(response[i])
			power := int(response[i+1])
			powers[antID] = power
		}
	}

	// Default if empty response
	if len(powers) == 0 {
		powers = map[int]int{1: 30, 2: 30, 3: 30, 4: 30}
	}

	return powers, nil
}

// ConfigureGPO sets GPO state
func (r *NRNReader) ConfigureGPO(gpoID int, state bool) error {
	payload := []byte{byte(gpoID)}
	if state {
		payload = append(payload, 0x01)
	} else {
		payload = append(payload, 0x00)
	}

	frame := r.BuildFrame(MIDConfigureGpo, payload)
	_, err := r.port.Write(frame)
	return err
}

// QueryGPI returns GPI state
func (r *NRNReader) QueryGPI(gpiID int) (bool, error) {
	payload := []byte{byte(gpiID)}
	frame := r.BuildFrame(MIDQueryGpi, payload)
	response, err := r.sendAndReceive(frame)
	if err != nil {
		return false, err
	}

	// Response format: [gpi_id, state]
	if len(response) >= 2 {
		return response[1] != 0x00, nil
	}

	return false, nil
}

// QueryReaderInformation returns reader device information
func (r *NRNReader) QueryReaderInformation() (*ReaderInfo, error) {
	frame := r.BuildFrame(MIDQueryInfo, nil)
	response, err := r.sendAndReceive(frame)
	if err != nil {
		return nil, err
	}

	info := &ReaderInfo{}

	// Parse TLV-style response data
	cursor := 0
	for cursor < len(response) {
		if cursor+2 > len(response) {
			break
		}

		pid := response[cursor]
		plen := int(response[cursor+1])
		cursor += 2

		if cursor+plen > len(response) {
			break
		}

		fieldData := response[cursor : cursor+plen]
		cursor += plen

		switch pid {
		case 0x01: // Serial Number
			info.SerialNumber = string(fieldData)
		case 0x02: // Power On Time (4 bytes)
			if plen >= 4 {
				info.PowerOnTimeSec = binary.BigEndian.Uint32(fieldData)
			}
		case 0x03: // Baseband Compile Time
			info.BasebandCompileTime = string(fieldData)
		case 0x04: // App Version
			info.AppVersion = string(fieldData)
		case 0x05: // OS Version
			info.OSVersion = string(fieldData)
		case 0x06: // App Compile Time
			info.AppCompileTime = string(fieldData)
		}
	}

	return info, nil
}

// QueryRFIDAbility returns reader capabilities
func (r *NRNReader) QueryRFIDAbility() (*RFIDAbility, error) {
	frame := r.BuildFrame(MIDQueryRfidAbility, nil)
	response, err := r.sendAndReceive(frame)
	if err != nil {
		return nil, err
	}

	ability := &RFIDAbility{
		MinPower:     0,
		MaxPower:     33,
		AntennaCount: 4,
		Frequencies:  []float64{},
	}

	// Parse TLV-style response data
	cursor := 0
	for cursor < len(response) {
		if cursor+2 > len(response) {
			break
		}

		pid := response[cursor]
		plen := int(response[cursor+1])
		cursor += 2

		if cursor+plen > len(response) {
			break
		}

		fieldData := response[cursor : cursor+plen]
		cursor += plen

		switch pid {
		case 0x01: // Min Power (1 byte)
			if plen >= 1 {
				ability.MinPower = int32(fieldData[0])
			}
		case 0x02: // Max Power (1 byte)
			if plen >= 1 {
				ability.MaxPower = int32(fieldData[0])
			}
		case 0x03: // Antenna Count (1 byte)
			if plen >= 1 {
				ability.AntennaCount = fieldData[0]
			}
		case 0x04: // Supported Frequencies (4 bytes each, KHz)
			for i := 0; i+3 < plen; i += 4 {
				freqKHz := binary.BigEndian.Uint32(fieldData[i : i+4])
				ability.Frequencies = append(ability.Frequencies, float64(freqKHz)/1000.0)
			}
		}
	}

	return ability, nil
}

// sendAndReceive sends a frame and waits for response
func (r *NRNReader) sendAndReceive(frame []byte) ([]byte, error) {
	r.mu.Lock()
	defer r.mu.Unlock()

	// Clear input buffer
	r.port.ResetInputBuffer()

	// Send frame
	_, err := r.port.Write(frame)
	if err != nil {
		return nil, fmt.Errorf("write error: %w", err)
	}

	// Read response with timeout
	buffer := make([]byte, 1024)
	n, err := r.port.Read(buffer)
	if err != nil {
		return nil, fmt.Errorf("read error: %w", err)
	}

	if n < 9 {
		return nil, fmt.Errorf("response too short: %d bytes", n)
	}

	// Parse frame
	parsed := r.ParseFrame(buffer[:n])
	if !parsed.Valid {
		return nil, fmt.Errorf("invalid response frame")
	}

	return parsed.Data, nil
}

// ParseFrame parses a raw protocol frame
func (r *NRNReader) ParseFrame(data []byte) ParsedFrame {
	frame := ParsedFrame{}

	if len(data) < 9 || data[0] != FrameHeader {
		frame.Valid = false
		return frame
	}

	frame.ProtoType = data[1]
	frame.ProtoVer = data[2]
	frame.RS485 = (data[3] & 0x80) != 0
	frame.Notify = (data[3] & 0x10) != 0
	frame.Category = data[4]
	frame.MID = data[5]
	frame.DataLength = binary.BigEndian.Uint16(data[6:8])

	if len(data) < int(8+frame.DataLength+2) {
		frame.Valid = false
		return frame
	}

	frame.Data = make([]byte, frame.DataLength)
	copy(frame.Data, data[8:8+frame.DataLength])

	crcOffset := 8 + int(frame.DataLength)
	frame.CRC = binary.BigEndian.Uint16(data[crcOffset : crcOffset+2])

	// Verify CRC
	calculatedCRC := CRC16CCITT(data[1:crcOffset])
	frame.Valid = calculatedCRC == frame.CRC

	return frame
}

