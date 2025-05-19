import json
import os
import sys
import time
import socketio
from serial import Serial
from serial.tools import list_ports
import re

SERVER_ROOT   = "wss://api.nextwaves.vn/"
DEVICE_ID     = "LAUNCHPAD"
DEVICE_TOKEN  = "123123"
SERIAL_PORT   = "COM15"
BAUD_RATE     = 115200
TIMEOUT       = 1

print("Connecting to serial port:", SERIAL_PORT)
# list all
ports = list_ports.comports()
for p in ports:
    print(p.device, p.description)

ser = Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)

# buffer giữ byte thừa cho các lần parse
_buffer = bytearray()

def parser(s):
    """
    Parse raw hex stream into list of tags, xử lý đủ cả frame dài lẫn ngắn.
    """
    global _buffer

    # chuẩn hoá input hex
    clean = re.sub(r"[^0-9A-Fa-f]", "", s).upper()
    if len(clean) & 1:
        clean = clean[:-1]
    chunk = bytes.fromhex(clean)

    # đẩy vào buffer
    _buffer.extend(chunk)

    # bảng tần số và RSSI (giữ nguyên từ bản gốc)
    FREQUENCY_TABLE = {
        "00": 865.00, "01": 865.50, "02": 866.00, "03": 866.50,
        "04": 867.00, "05": 867.50, "06": 868.00, "07": 902.00,
        "08": 902.50, "09": 903.00, "0A": 903.50, "0B": 904.00,
        "0C": 904.50, "0D": 905.00, "0E": 905.50, "0F": 906.00,
        "10": 906.50, "11": 907.00, "12": 907.50, "13": 908.00,
        "14": 908.50, "15": 909.00, "16": 909.50, "17": 910.00,
        "18": 910.50, "19": 911.00, "1A": 911.50, "1B": 912.00,
        "1C": 912.50, "1D": 913.00, "1E": 913.50, "1F": 914.00,
        "20": 914.50, "21": 915.00, "22": 915.50, "23": 916.00,
        "24": 916.50, "25": 917.00, "26": 917.50, "27": 918.00,
        "28": 918.50, "29": 919.00, "2A": 919.50, "2B": 920.00,
        "2C": 920.50, "2D": 921.00, "2E": 921.50, "2F": 922.00,
        "30": 922.50, "31": 923.00, "32": 923.50, "33": 924.00,
        "34": 924.50, "35": 925.00, "36": 925.50, "37": 926.00,
        "38": 926.50, "39": 927.00, "3A": 927.50, "3B": 928.00,
    }
    RSSI_DBM = {
        "6E": -19, "6D": -20, "6C": -21, "6B": -22, "6A": -23,
        "69": -24, "68": -25, "67": -26, "66": -27, "65": -28,
        "64": -29, "63": -30, "62": -31, "61": -32, "60": -33,
        "5F": -34, "5E": -35, "5D": -36, "5C": -37, "5B": -38,
        "5A": -39, "59": -41, "58": -42, "57": -43, "56": -44,
        "55": -45, "54": -46, "53": -47, "52": -48, "51": -49,
        "50": -50, "4F": -51, "4E": -52, "4D": -53, "4C": -54,
        "4B": -55, "4A": -56, "49": -57, "48": -58, "47": -59,
        "46": -60, "45": -61, "44": -62, "43": -63, "42": -64,
        "41": -65, "40": -66, "3F": -67, "3E": -68, "3D": -69,
        "3C": -70, "3B": -71, "3A": -72, "39": -73, "38": -74,
        "37": -75, "36": -76, "35": -77, "34": -78, "33": -79,
        "32": -80, "31": -81, "30": -82, "2F": -83, "2E": -84,
        "2D": -85, "2C": -86, "2B": -87, "2A": -88, "29": -89,
        "28": -90, "27": -91, "26": -92, "25": -93, "24": -94,
        "23": -95, "22": -96, "21": -97, "20": -98, "1F": -99,
    }

    tags = []
    i = 0
    # tách từng frame hoàn chỉnh
    while i + 2 <= len(_buffer):
        if _buffer[i] != 0xA0:
            i += 1
            continue
        length = _buffer[i + 1]
        frame_len = length + 2
        if i + frame_len > len(_buffer):
            break  # chờ đủ byte
        frame = bytes(_buffer[i : i + frame_len])

        # chỉ parse inventory frames (cmd=0x8A)
        if len(frame) >= 5 and frame[3] == 0x8A:
            freq_ant = frame[4]
            antenna  = (freq_ant & 0x03) + 1
            ch_idx   = freq_ant >> 2
            frequency = FREQUENCY_TABLE.get(f"{ch_idx:02X}", 0)

            pc_low    = frame[5]
            epc_words = (pc_low >> 3) & 0x1F
            epc_len   = epc_words * 2

            epc_start = 7
            epc_end   = epc_start + epc_len
            # giữ lại đủ 2 byte cuối RSSI+CRC
            if epc_end > len(frame) - 2:
                epc_end = len(frame) - 2

            epc = frame[epc_start:epc_end].hex().upper()
            if epc:
                rssi_byte = frame[-2]
                rssi = RSSI_DBM.get(f"{rssi_byte:02X}", -100)
                tags.append({
                    "epc":       epc,
                    "antenna":   antenna,
                    "frequency": frequency,
                    "rssi":      rssi,
                })

        i += frame_len

    # giữ lại phần buffer chưa parse
    _buffer[:] = _buffer[i:]
    print("Parsed tags:", tags)
    return tags


def restart_program(delay: float = 3.0):
    print(f"🔄 Restarting script in {delay:.0f}s …")
    time.sleep(delay)
    os.execl(sys.executable, sys.executable, *sys.argv)


def emit_json(event: str, payload: dict):
    """Helper to emit JSON via Socket.IO."""
    sio.emit(event, json.dumps(payload))


def set_output_power(p1=30, p2=30, p3=30, p4=10):
    for p in (p1, p2, p3, p4):
        if not 0 <= p <= 33:
            raise ValueError("Power values must be between 0-33 dBm")
    cmd = bytearray([0xA0, 0x07, 0x76, p1, p2, p3, p4])
    cmd.append(sum(cmd) & 0xFF)
    ser.write(cmd)
    time.sleep(0.15)
    if ser.in_waiting:
        return ser.read(ser.in_waiting)[:4] == b"\xA0\x04\x76\x00"
    return True


sio = socketio.Client(reconnection=False)


@sio.event
def connect():
    print("✅ Connected to WebSocket server. sid =", sio.sid)


@sio.event
def disconnect():
    print("🔌 Disconnected from server.")
    restart_program()


@sio.event
def connect_error(data):
    print("❌ Handshake failed:", data)
    restart_program()


def read_epc_data():
    poll_cmd = bytes.fromhex("A00DFF8A00010101020103010101BE")
    while True:
        ser.write(poll_cmd)
        time.sleep(0.1)
        if ser.in_waiting:
            raw_hex = ser.read(ser.in_waiting).hex().upper()
            parsed_tags = parser(raw_hex)
            for tag in parsed_tags:
                payload = {
                    "device_id": DEVICE_ID,
                    "epc":       tag["epc"],
                    "rssi":      tag["rssi"],
                    "frequency": tag["frequency"],
                    "raw":       raw_hex,
                }
                emit_json("message", payload)
                print(payload)
                emit_json("ack", {"epc": tag["epc"], "rssi": tag["rssi"]})
        sio.sleep(0.2)


if __name__ == "__main__":
    try:
        sio.connect(
            SERVER_ROOT,
            headers={"device-id": DEVICE_ID, "device-token": DEVICE_TOKEN},
            transports=["websocket"],
            socketio_path="socket.io",
        )
        print("🔍 Available serial ports:", [p.device for p in list_ports.comports()])
        print("⚙️  Setting reader output power …")
        if not set_output_power():
            print("❌ Failed to set power. Exiting.")
            sys.exit(1)
        print("📡 Reader ready — starting EPC stream (Ctrl-C to quit)")
        read_epc_data()
    except KeyboardInterrupt:
        print("\n🛑 Terminated by user.")
    except Exception as e:
        print("❌ Fatal error:", e)
    finally:
        ser.close()
        if sio.connected:
            sio.disconnect()
