/**
 * Nation RFID Reader SDK for C++ - Implementation
 * 
 * @version 1.0.0
 * @author Nextwaves
 */

#include "nrn.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>

// Platform-specific serial includes
#ifdef _WIN32
    #include <windows.h>
#else
    #include <fcntl.h>
    #include <termios.h>
    #include <unistd.h>
    #include <errno.h>
#endif

namespace nrn {

// === Utility Functions Implementation ===

uint16_t crc16_ccitt(const std::vector<uint8_t>& data) {
    uint16_t crc = CRC16_CCITT_INIT;
    for (uint8_t byte : data) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_CCITT_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

std::string bytes_to_hex(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (uint8_t byte : data) {
        oss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(byte);
    }
    return oss.str();
}

uint32_t build_antenna_mask(const std::vector<int>& antennas) {
    uint32_t mask = 0;
    for (int ant_id : antennas) {
        if (ant_id >= 1 && ant_id <= 32) {
            mask |= (1 << (ant_id - 1));
        }
    }
    return mask;
}

// === NRNReader Implementation ===

class NRNReader::Impl {
public:
#ifdef _WIN32
    HANDLE serial_handle = INVALID_HANDLE_VALUE;
#else
    int serial_fd = -1;
#endif
    std::atomic<bool> running{false};
    std::thread inventory_thread;
    std::mutex mutex;
    TagCallback tag_callback;
};

NRNReader::NRNReader(const std::string& port, int baudrate, int timeout_ms)
    : pimpl_(std::make_unique<Impl>())
    , port_(port)
    , baudrate_(baudrate)
    , timeout_ms_(timeout_ms)
    , antenna_mask_(0x00000001)
{}

NRNReader::~NRNReader() {
    close();
}

bool NRNReader::open() {
#ifdef _WIN32
    pimpl_->serial_handle = CreateFileA(
        port_.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        0,
        nullptr
    );
    
    if (pimpl_->serial_handle == INVALID_HANDLE_VALUE) {
        log("ERROR", "Failed to open port: " + port_);
        return false;
    }
    
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    GetCommState(pimpl_->serial_handle, &dcb);
    dcb.BaudRate = baudrate_;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(pimpl_->serial_handle, &dcb);
    
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = timeout_ms_;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(pimpl_->serial_handle, &timeouts);
    
#else
    pimpl_->serial_fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (pimpl_->serial_fd < 0) {
        log("ERROR", "Failed to open port: " + port_ + " - " + strerror(errno));
        return false;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(pimpl_->serial_fd, &tty) != 0) {
        log("ERROR", "Failed to get terminal attributes");
        ::close(pimpl_->serial_fd);
        pimpl_->serial_fd = -1;
        return false;
    }
    
    // Set baud rate
    speed_t baud;
    switch (baudrate_) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default: baud = B115200;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    
    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    
    // Timeout
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = timeout_ms_ / 100;
    
    if (tcsetattr(pimpl_->serial_fd, TCSANOW, &tty) != 0) {
        log("ERROR", "Failed to set terminal attributes");
        ::close(pimpl_->serial_fd);
        pimpl_->serial_fd = -1;
        return false;
    }
#endif
    
    log("INFO", "Opened port: " + port_);
    return true;
}

void NRNReader::close() {
    stop_inventory();
    
#ifdef _WIN32
    if (pimpl_->serial_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(pimpl_->serial_handle);
        pimpl_->serial_handle = INVALID_HANDLE_VALUE;
    }
#else
    if (pimpl_->serial_fd >= 0) {
        ::close(pimpl_->serial_fd);
        pimpl_->serial_fd = -1;
    }
#endif
    
    log("INFO", "Closed port");
}

bool NRNReader::is_open() const {
#ifdef _WIN32
    return pimpl_->serial_handle != INVALID_HANDLE_VALUE;
#else
    return pimpl_->serial_fd >= 0;
#endif
}

bool NRNReader::connect_and_initialize() {
    if (!is_open()) {
        if (!open()) {
            return false;
        }
    }
    
    // Send connection confirmation
    std::vector<uint8_t> payload = {};
    auto frame = build_frame(MID::CONFIRM_CONNECTION, payload);
    
    // Write frame
#ifdef _WIN32
    DWORD written;
    WriteFile(pimpl_->serial_handle, frame.data(), frame.size(), &written, nullptr);
#else
    write(pimpl_->serial_fd, frame.data(), frame.size());
#endif
    
    log("INFO", "Reader initialized");
    return true;
}

std::vector<uint8_t> NRNReader::build_epc_read_payload(uint32_t antenna_mask, bool continuous, bool include_tid) {
    if (antenna_mask == 0) {
        antenna_mask = 0x00000001;
    }
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>((antenna_mask >> 24) & 0xFF),
        static_cast<uint8_t>((antenna_mask >> 16) & 0xFF),
        static_cast<uint8_t>((antenna_mask >> 8) & 0xFF),
        static_cast<uint8_t>(antenna_mask & 0xFF),
        static_cast<uint8_t>(continuous ? 0x01 : 0x00),
    };
    
    if (include_tid) {
        data.push_back(0x02);  // PID 0x02: TID reading parameters
        data.push_back(0x00);  // Byte 0: TID reading mode (0: self-adaptable)
        data.push_back(0x00);  // Byte 1: Word length (0x00 = Auto / Max)
    }
    
    return data;
}

std::vector<uint8_t> NRNReader::build_frame(MID mid, const std::vector<uint8_t>& payload) {
    uint8_t category = (static_cast<uint16_t>(mid) >> 8) & 0xFF;
    uint8_t mid_byte = static_cast<uint16_t>(mid) & 0xFF;
    
    std::vector<uint8_t> frame;
    frame.push_back(FRAME_HEADER);
    
    // PCW: Proto Type, Proto Version, Flags, Category
    frame.push_back(0x00);  // Proto Type
    frame.push_back(0x01);  // Proto Version
    frame.push_back(0x00);  // Flags
    frame.push_back(category);
    
    // MID
    frame.push_back(mid_byte);
    
    // Length
    uint16_t len = payload.size();
    frame.push_back((len >> 8) & 0xFF);
    frame.push_back(len & 0xFF);
    
    // Payload
    frame.insert(frame.end(), payload.begin(), payload.end());
    
    // CRC (computed over everything except header and CRC itself)
    std::vector<uint8_t> crc_data(frame.begin() + 1, frame.end());
    uint16_t crc = crc16_ccitt(crc_data);
    frame.push_back((crc >> 8) & 0xFF);
    frame.push_back(crc & 0xFF);
    
    return frame;
}

TagData NRNReader::parse_epc(const std::vector<uint8_t>& data) {
    TagData tag;
    
    if (data.size() < 5) {
        return tag;
    }
    
    // 1. Read EPC (Mandatory)
    uint16_t epc_len = (data[0] << 8) | data[1];
    if (data.size() < 2 + epc_len + 3) {
        return tag;
    }
    
    std::vector<uint8_t> epc_bytes(data.begin() + 2, data.begin() + 2 + epc_len);
    tag.epc = bytes_to_hex(epc_bytes);
    
    // 2. Read PC (Mandatory - 2 bytes)
    std::vector<uint8_t> pc_bytes(data.begin() + 2 + epc_len, data.begin() + 2 + epc_len + 2);
    tag.pc = bytes_to_hex(pc_bytes);
    
    // 3. Read Antenna ID (Mandatory - 1 byte)
    tag.antenna_id = data[2 + epc_len + 2];
    
    // 4. Parse optional PIDs starting after Antenna
    size_t cursor = 2 + epc_len + 3;
    
    while (cursor < data.size()) {
        uint8_t pid = data[cursor];
        cursor++;
        
        if (pid == 0x01) {  // RSSI (1 byte)
            if (cursor < data.size()) {
                tag.rssi = calculate_rssi(data[cursor]);
                cursor++;
            } else break;
        }
        else if (pid == 0x02) {  // Reading Result (1 byte)
            if (cursor < data.size()) {
                cursor++;
            } else break;
        }
        else if (pid == 0x03) {  // TID DATA (Variable length)
            if (cursor + 1 < data.size()) {
                uint16_t tid_len = (data[cursor] << 8) | data[cursor + 1];
                cursor += 2;
                if (cursor + tid_len <= data.size()) {
                    std::vector<uint8_t> tid_bytes(data.begin() + cursor, data.begin() + cursor + tid_len);
                    tag.tid = bytes_to_hex(tid_bytes);
                    cursor += tid_len;
                } else break;
            } else break;
        }
        else if (pid == 0x04 || pid == 0x05) {  // Tag/Reserve data
            if (cursor + 1 < data.size()) {
                uint16_t byte_len = (data[cursor] << 8) | data[cursor + 1];
                cursor += 2 + byte_len;
            } else break;
        }
        else if (pid == 0x06) {  // Sub-antenna (1 byte)
            cursor++;
        }
        else if (pid == 0x07) {  // UTC time (8 bytes)
            cursor += 8;
        }
        else if (pid == 0x08) {  // Frequency (4 bytes, KHz)
            if (cursor + 3 < data.size()) {
                uint32_t freq_khz = (data[cursor] << 24) | (data[cursor + 1] << 16) |
                                   (data[cursor + 2] << 8) | data[cursor + 3];
                tag.frequency = freq_khz / 1000.0;
                cursor += 4;
            } else break;
        }
        else if (pid == 0x09) {  // Phase (1 byte, 0~128)
            if (cursor < data.size()) {
                uint8_t phase_raw = data[cursor];
                tag.phase = (phase_raw / 128.0) * 2 * 3.14159265359;
                cursor++;
            } else break;
        }
        else {
            // Unknown PID - skip
            continue;
        }
    }
    
    return tag;
}

bool NRNReader::start_inventory(uint32_t antenna_mask, TagCallback callback, bool include_tid) {
    if (!is_open()) {
        log("ERROR", "Port not open");
        return false;
    }
    
    antenna_mask_ = antenna_mask;
    pimpl_->tag_callback = callback;
    pimpl_->running = true;
    
    auto payload = build_epc_read_payload(antenna_mask, true, include_tid);
    auto frame = build_frame(MID::READ_EPC_TAG, payload);
    
#ifdef _WIN32
    DWORD written;
    WriteFile(pimpl_->serial_handle, frame.data(), frame.size(), &written, nullptr);
#else
    write(pimpl_->serial_fd, frame.data(), frame.size());
#endif
    
    log("INFO", "Inventory started");
    return true;
}

bool NRNReader::stop_inventory() {
    pimpl_->running = false;
    
    if (!is_open()) {
        return true;
    }
    
    auto frame = build_frame(MID::STOP_INVENTORY, {});
    
#ifdef _WIN32
    DWORD written;
    WriteFile(pimpl_->serial_handle, frame.data(), frame.size(), &written, nullptr);
#else
    write(pimpl_->serial_fd, frame.data(), frame.size());
#endif
    
    log("INFO", "Inventory stopped");
    return true;
}

bool NRNReader::configure_power(const std::map<int, int>& powers, bool persist) {
    // Build power configuration payload
    std::vector<uint8_t> payload;
    payload.push_back(persist ? 0x01 : 0x00);
    
    for (const auto& [ant_id, power] : powers) {
        payload.push_back(static_cast<uint8_t>(ant_id));
        payload.push_back(static_cast<uint8_t>(power));
    }
    
    auto frame = build_frame(MID::CONFIGURE_READER_POWER, payload);
    
#ifdef _WIN32
    DWORD written;
    WriteFile(pimpl_->serial_handle, frame.data(), frame.size(), &written, nullptr);
#else
    write(pimpl_->serial_fd, frame.data(), frame.size());
#endif
    
    log("INFO", "Power configured");
    return true;
}

std::map<int, int> NRNReader::query_power() {
    auto frame = build_frame(MID::QUERY_READER_POWER, {});
    auto response = send_and_receive(frame);
    
    std::map<int, int> powers;
    // Response format: [persistence byte] + [ant_id, power] pairs
    if (response.size() >= 1) {
        for (size_t i = 1; i + 1 < response.size(); i += 2) {
            int ant_id = response[i];
            int power = response[i + 1];
            powers[ant_id] = power;
        }
    }
    
    // Default if empty response
    if (powers.empty()) {
        powers = {{1, 30}, {2, 30}, {3, 30}, {4, 30}};
    }
    
    return powers;
}

void NRNReader::set_log_callback(LogCallback callback) {
    log_callback_ = callback;
}

void NRNReader::log(const std::string& level, const std::string& message) {
    if (log_callback_) {
        log_callback_(level, message);
    }
}

ReaderInfo NRNReader::query_reader_information() {
    auto frame = build_frame(MID::QUERY_INFO, {});
    auto response = send_and_receive(frame);
    
    ReaderInfo info;
    
    // Parse TLV-style response data
    size_t cursor = 0;
    while (cursor < response.size()) {
        if (cursor + 2 > response.size()) {
            break;
        }
        
        uint8_t pid = response[cursor];
        uint8_t plen = response[cursor + 1];
        cursor += 2;
        
        if (cursor + plen > response.size()) {
            break;
        }
        
        std::vector<uint8_t> field_data(response.begin() + cursor, response.begin() + cursor + plen);
        cursor += plen;
        
        switch (pid) {
            case 0x01: // Serial Number
                info.serial_number = std::string(field_data.begin(), field_data.end());
                break;
            case 0x02: // Power On Time (4 bytes)
                if (plen >= 4) {
                    info.power_on_time_sec = (field_data[0] << 24) | (field_data[1] << 16) |
                                             (field_data[2] << 8) | field_data[3];
                }
                break;
            case 0x03: // Baseband Compile Time
                info.baseband_compile_time = std::string(field_data.begin(), field_data.end());
                break;
            case 0x04: // App Version
                info.app_version = std::string(field_data.begin(), field_data.end());
                break;
            case 0x05: // OS Version
                info.os_version = std::string(field_data.begin(), field_data.end());
                break;
            case 0x06: // App Compile Time
                info.app_compile_time = std::string(field_data.begin(), field_data.end());
                break;
        }
    }
    
    return info;
}

RFIDAbility NRNReader::query_rfid_ability() {
    auto frame = build_frame(MID::QUERY_RFID_ABILITY, {});
    auto response = send_and_receive(frame);
    
    RFIDAbility ability = {0, 33, 4, {}};
    
    // Parse TLV-style response data
    size_t cursor = 0;
    while (cursor < response.size()) {
        if (cursor + 2 > response.size()) {
            break;
        }
        
        uint8_t pid = response[cursor];
        uint8_t plen = response[cursor + 1];
        cursor += 2;
        
        if (cursor + plen > response.size()) {
            break;
        }
        
        std::vector<uint8_t> field_data(response.begin() + cursor, response.begin() + cursor + plen);
        cursor += plen;
        
        switch (pid) {
            case 0x01: // Min Power (1 byte)
                if (plen >= 1) {
                    ability.min_power = field_data[0];
                }
                break;
            case 0x02: // Max Power (1 byte)
                if (plen >= 1) {
                    ability.max_power = field_data[0];
                }
                break;
            case 0x03: // Antenna Count (1 byte)
                if (plen >= 1) {
                    ability.antenna_count = field_data[0];
                }
                break;
            case 0x04: // Supported Frequencies (4 bytes each, KHz)
                for (size_t i = 0; i + 3 < plen; i += 4) {
                    uint32_t freq_khz = (field_data[i] << 24) | (field_data[i + 1] << 16) |
                                       (field_data[i + 2] << 8) | field_data[i + 3];
                    ability.frequencies.push_back(freq_khz / 1000.0);
                }
                break;
        }
    }
    
    return ability;
}

bool NRNReader::configure_gpo(int gpo_id, bool state) {
    std::vector<uint8_t> payload = {
        static_cast<uint8_t>(gpo_id),
        static_cast<uint8_t>(state ? 0x01 : 0x00)
    };
    auto frame = build_frame(MID::CONFIGURE_GPO, payload);
    
#ifdef _WIN32
    DWORD written;
    WriteFile(pimpl_->serial_handle, frame.data(), frame.size(), &written, nullptr);
#else
    write(pimpl_->serial_fd, frame.data(), frame.size());
#endif
    
    return true;
}

bool NRNReader::query_gpi(int gpi_id) {
    std::vector<uint8_t> payload = {static_cast<uint8_t>(gpi_id)};
    auto frame = build_frame(MID::QUERY_GPI, payload);
    auto response = send_and_receive(frame);
    
    // Response format: [gpi_id, state]
    if (response.size() >= 2) {
        return response[1] != 0x00;
    }
    
    return false;
}

std::vector<uint8_t> NRNReader::send_and_receive(const std::vector<uint8_t>& frame) {
    std::lock_guard<std::mutex> lock(pimpl_->mutex);
    
    // Send frame
#ifdef _WIN32
    DWORD written;
    WriteFile(pimpl_->serial_handle, frame.data(), frame.size(), &written, nullptr);
    
    // Read response
    std::vector<uint8_t> buffer(1024);
    DWORD bytes_read;
    ReadFile(pimpl_->serial_handle, buffer.data(), buffer.size(), &bytes_read, nullptr);
    buffer.resize(bytes_read);
#else
    write(pimpl_->serial_fd, frame.data(), frame.size());
    
    // Read response
    std::vector<uint8_t> buffer(1024);
    ssize_t bytes_read = read(pimpl_->serial_fd, buffer.data(), buffer.size());
    if (bytes_read > 0) {
        buffer.resize(bytes_read);
    } else {
        buffer.clear();
    }
#endif
    
    if (buffer.size() < 9) {
        return {};
    }
    
    // Parse frame
    auto parsed = parse_frame(buffer);
    if (!parsed.valid) {
        return {};
    }
    
    return parsed.data;
}

ParsedFrame NRNReader::parse_frame(const std::vector<uint8_t>& data) {
    ParsedFrame frame{};
    
    if (data.size() < 9 || data[0] != FRAME_HEADER) {
        frame.valid = false;
        return frame;
    }
    
    frame.proto_type = data[1];
    frame.proto_ver = data[2];
    frame.rs485 = (data[3] & 0x80) != 0;
    frame.notify = (data[3] & 0x10) != 0;
    frame.category = data[4];
    frame.mid = data[5];
    frame.data_length = (data[6] << 8) | data[7];
    
    if (data.size() < 8 + frame.data_length + 2) {
        frame.valid = false;
        return frame;
    }
    
    frame.data = std::vector<uint8_t>(data.begin() + 8, data.begin() + 8 + frame.data_length);
    
    size_t crc_offset = 8 + frame.data_length;
    frame.crc = (data[crc_offset] << 8) | data[crc_offset + 1];
    
    // Verify CRC
    std::vector<uint8_t> crc_data(data.begin() + 1, data.begin() + crc_offset);
    uint16_t calculated_crc = crc16_ccitt(crc_data);
    frame.valid = (calculated_crc == frame.crc);
    
    return frame;
}

} // namespace nrn

