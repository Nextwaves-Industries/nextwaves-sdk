/**
 * Nation RFID Reader SDK for C++
 * 
 * A comprehensive serial communication SDK for NRN RFID readers.
 * 
 * @version 1.0.0
 * @author Nextwaves
 */

#ifndef NRN_SDK_HPP
#define NRN_SDK_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <map>
#include <optional>
#include <memory>

namespace nrn {

// === Constants ===

constexpr uint8_t FRAME_HEADER = 0x5A;
constexpr uint16_t CRC16_CCITT_INIT = 0x0000;
constexpr uint16_t CRC16_CCITT_POLY = 0x8005;

// === Message IDs ===
enum class MID : uint16_t {
    // Reader Configuration
    QUERY_INFO = 0x0100,
    CONFIRM_CONNECTION = 0x12,
    
    // RFID Inventory
    READ_EPC_TAG = 0x0210,
    PHASE_INVENTORY = 0x0214,
    STOP_INVENTORY = 0x02FF,
    WRITE_EPC_TAG = 0x0211,
    
    // Error Handling
    ERROR_NOTIFICATION = 0x00,
    
    // RFID Baseband
    CONFIG_BASEBAND = 0x020B,
    QUERY_BASEBAND = 0x020C,
    
    // Power Control
    CONFIGURE_READER_POWER = 0x0201,
    QUERY_READER_POWER = 0x0202,
    READER_POWER_CALIBRATION = 0x0103,
    QUERY_POWER_CALIBRATION = 0x0104,
    
    // Filter Settings
    SET_FILTER_SETTINGS = 0x0209,
    QUERY_FILTER_SETTINGS = 0x020A,
    
    // RF Band & Frequency
    SET_RF_BAND = 0x0203,
    QUERY_RF_BAND = 0x0204,
    SET_WORKING_FREQUENCY = 0x0205,
    QUERY_WORKING_FREQUENCY = 0x0206,
    
    // RFID Ability
    QUERY_RFID_ABILITY = 0x1000,
    
    // Buzzer Control
    BUZZER_SWITCH = 0x011E,
    
    // GPIO Commands
    CONFIGURE_GPO = 0x0109,
    QUERY_GPI = 0x010A,
    CONFIGURE_GPI_TRIGGER = 0x010B,
    QUERY_GPI_TRIGGER = 0x010C,
};

// === Beeper Modes ===
enum class BeeperMode : uint8_t {
    QUIET = 0x00,
    BEEP_AFTER_INVENTORY = 0x01,
    BEEP_AFTER_TAG = 0x02,
};

// === RF Profiles ===
struct RFProfile {
    int id;
    std::string name;
    std::string description;
};

const std::map<int, RFProfile> RF_PROFILES = {
    {0, {0, "Profile 0", "Default baseband profile"}},
    {1, {1, "Profile 1", "High performance profile"}},
    {2, {2, "Profile 2", "Dense tag profile"}},
};

// === Data Structures ===

struct TagData {
    std::string epc;
    std::string pc;
    uint8_t antenna_id;
    std::optional<int> rssi;
    std::optional<std::string> tid;
    std::optional<double> phase;
    std::optional<double> frequency;
};

struct ReaderInfo {
    std::string serial_number;
    uint32_t power_on_time_sec;
    std::string baseband_compile_time;
    std::string app_version;
    std::string os_version;
    std::string app_compile_time;
};

struct RFIDAbility {
    int min_power;
    int max_power;
    int antenna_count;
    std::vector<double> frequencies;
};

struct ParsedFrame {
    bool valid;
    uint8_t proto_type;
    uint8_t proto_ver;
    bool rs485;
    bool notify;
    uint16_t pcw;
    uint8_t category;
    uint8_t mid;
    uint16_t data_length;
    std::vector<uint8_t> data;
    uint16_t crc;
};

// === Callback Types ===
using TagCallback = std::function<void(const TagData&)>;
using LogCallback = std::function<void(const std::string& level, const std::string& message)>;

// === Utility Functions ===

/**
 * Calculate CRC16-CCITT checksum
 */
uint16_t crc16_ccitt(const std::vector<uint8_t>& data);

/**
 * Calculate RSSI in dBm from raw byte value
 * Formula: -100 + round((rssiRaw * 70) / 255)
 */
inline int calculate_rssi(uint8_t rssi_raw) {
    return -100 + static_cast<int>((rssi_raw * 70 + 127) / 255);
}

/**
 * Calculate frequency in MHz from channel index
 * Formula: 920.0 + chIdx * 0.5
 */
inline double calculate_frequency(int ch_idx) {
    return 920.0 + ch_idx * 0.5;
}

/**
 * Convert bytes to hex string
 */
std::string bytes_to_hex(const std::vector<uint8_t>& data);

/**
 * Build antenna mask from list of antenna IDs
 */
uint32_t build_antenna_mask(const std::vector<int>& antennas);

// === NRNReader Class ===

class NRNReader {
public:
    /**
     * Constructor
     * @param port Serial port path (e.g., "/dev/ttyUSB0", "COM3")
     * @param baudrate Baud rate (default: 115200)
     * @param timeout_ms Read timeout in milliseconds (default: 500)
     */
    NRNReader(const std::string& port, int baudrate = 115200, int timeout_ms = 500);
    
    ~NRNReader();

    // === Connection ===
    
    /**
     * Open serial connection
     * @return true if successful
     */
    bool open();
    
    /**
     * Close serial connection
     */
    void close();
    
    /**
     * Check if connection is open
     */
    bool is_open() const;
    
    /**
     * Connect and initialize reader
     * @return true if successful
     */
    bool connect_and_initialize();

    // === Reader Information ===
    
    /**
     * Query reader information
     */
    ReaderInfo query_reader_information();
    
    /**
     * Query RFID capabilities
     */
    RFIDAbility query_rfid_ability();

    // === Inventory Operations ===
    
    /**
     * Start continuous inventory
     * @param antenna_mask Bitmask of antennas to use
     * @param callback Function called for each tag
     * @param include_tid Whether to read TID data
     * @return true if started successfully
     */
    bool start_inventory(uint32_t antenna_mask, TagCallback callback, bool include_tid = false);
    
    /**
     * Stop current inventory operation
     * @return true if stopped successfully
     */
    bool stop_inventory();

    // === Power Control ===
    
    /**
     * Configure antenna power
     * @param powers Map of antenna ID to power in dBm
     * @param persist Save after power-down
     * @return true if successful
     */
    bool configure_power(const std::map<int, int>& powers, bool persist = false);
    
    /**
     * Query current power settings
     * @return Map of antenna ID to power in dBm
     */
    std::map<int, int> query_power();

    // === GPIO Control ===
    
    /**
     * Configure GPO state
     * @param gpo_id GPO port (1-4)
     * @param state Output state (true = high)
     * @return true if successful
     */
    bool configure_gpo(int gpo_id, bool state);
    
    /**
     * Query GPI state
     * @param gpi_id GPI port (1-4)
     * @return GPI state (true = high)
     */
    bool query_gpi(int gpi_id);

    // === Frame Building ===
    
    /**
     * Build EPC read payload
     * @param antenna_mask Antenna bitmask
     * @param continuous Continuous mode
     * @param include_tid Include TID reading
     */
    std::vector<uint8_t> build_epc_read_payload(uint32_t antenna_mask, bool continuous = true, bool include_tid = false);
    
    /**
     * Build protocol frame
     * @param mid Message ID
     * @param payload Data payload
     */
    std::vector<uint8_t> build_frame(MID mid, const std::vector<uint8_t>& payload);

    // === Frame Parsing ===
    
    /**
     * Parse received frame
     */
    ParsedFrame parse_frame(const std::vector<uint8_t>& data);
    
    /**
     * Parse EPC tag data from inventory response
     */
    TagData parse_epc(const std::vector<uint8_t>& data);

    // === Logging ===
    void set_log_callback(LogCallback callback);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    
    std::string port_;
    int baudrate_;
    int timeout_ms_;
    LogCallback log_callback_;
    uint32_t antenna_mask_;
    
    void log(const std::string& level, const std::string& message);
    std::vector<uint8_t> send_and_receive(const std::vector<uint8_t>& frame);
};

} // namespace nrn

#endif // NRN_SDK_HPP
