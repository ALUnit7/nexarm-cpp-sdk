#pragma once
// NexArm C++ SDK - Windows implementation (Linux port: replace serial backend)
// Protocol: FF FF [ID] [LEN] [CMD] [PAYLOAD...] [CHECKSUM]
// Checksum: ~(ID + LEN + CMD + sum(PAYLOAD)) & 0xFF
// Baud: 1000000, 8N1

#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
// Linux: replace with termios fd
using HANDLE = int;
#define INVALID_HANDLE_VALUE (-1)
#endif

// ── Command codes ────────────────────────────────────────────────────────────
namespace NexArmCmd {
    constexpr uint8_t FIRMWARE_VERSION = 0x01;
    constexpr uint8_t BATTERY_LEVEL    = 0x02;
    constexpr uint8_t ACTION_GROUP_RUN = 0x03;
    constexpr uint8_t ACTION_GROUP_STOP= 0x04;
    constexpr uint8_t COORDINATE_SET   = 0x08;
    constexpr uint8_t BUZZER_SET       = 0x09;
    constexpr uint8_t GET_CUR_COORDS   = 0x0B;
    constexpr uint8_t STEPPER_RUN      = 0x13;
    constexpr uint8_t ACTION_GROUP_ERASE=0x17;
    constexpr uint8_t SET_ESPNOW_CH    = 0x1E;
    constexpr uint8_t SET_GLOBAL_ACC   = 0x1F;
    constexpr uint8_t ESPNOW_SYNC      = 0x21;
    constexpr uint8_t MECANUM_CTRL     = 0x22;
    constexpr uint8_t ARM_MOVE_INC     = 0x32;
}

constexpr uint8_t NEXARM_SYSTEM_ID = 0xFF;

// ── Data structures ──────────────────────────────────────────────────────────
struct NexArmPacket {
    uint8_t              id;
    uint8_t              length;
    uint8_t              cmd;
    std::vector<uint8_t> payload;
};

struct NexArmCoords {
    int16_t  x, y, z;            // mm
    float    pitch;               // degrees (raw / 10.0)
    int16_t  roll;                // degrees
    int16_t  claw;                // degrees
    int16_t  servo[6];            // raw servo positions (0 if unavailable)
    bool     has_full;            // true when pitch/roll/claw/servo are valid
};

// ── Client ───────────────────────────────────────────────────────────────────
class NexArmClient {
public:
    explicit NexArmClient(const std::string& port, uint32_t baud = 1000000);
    ~NexArmClient();

    void open();
    void close();

    // Build & send a frame; no reply expected
    void send(uint8_t id, uint8_t cmd, const uint8_t* payload = nullptr, uint8_t plen = 0);

    // Send and wait for matching reply (throws std::runtime_error on timeout)
    NexArmPacket request(uint8_t id, uint8_t cmd,
                         const uint8_t* payload, uint8_t plen,
                         uint8_t expect_cmd, uint8_t expect_id,
                         int timeout_ms = 500);

    // Read next packet from stream (blocking up to timeout_ms)
    NexArmPacket read_packet(int timeout_ms = 200);

    // High-level API ──────────────────────────────────────────────────────────

    // Read firmware version string, e.g. "1.0.0"
    std::string get_firmware_version(int timeout_ms = 500);

    // Read battery voltage in mV
    uint16_t get_battery_voltage(int timeout_ms = 500);

    // Absolute move: pitch in degrees, x/y/z in mm, duration in ms
    void set_pose(float x, float y, float z, float pitch, float roll, float claw, uint16_t duration_ms);

    // Incremental move
    void move_increment(float dx, float dy, float dz, float dpitch, float droll, float dclaw, uint16_t duration_ms);

    // Query short coords reply (x/y/z only)
    NexArmCoords get_current_coords(int timeout_ms = 500);

    // Wait for the auto-broadcast long frame (x/y/z/pitch/roll/claw/servos)
    // The arm broadcasts this at ~10 Hz without being asked.
    NexArmCoords get_full_coords(int timeout_ms = 1000);

    // Buzzer: on_ms, off_ms, times, frequency_hz
    void set_buzzer(uint32_t on_ms, uint32_t off_ms, uint16_t times, uint16_t frequency);

    // Servo direct control (servo_id 1-6, position raw, speed)
    void servo_set_position(uint8_t servo_id, uint16_t position);

    // Flush receive buffer
    void flush_rx();

private:
    std::string m_port;
    uint32_t    m_baud;
    HANDLE      m_handle;

    // Low-level serial read/write
    int  serial_read(uint8_t* buf, int len, int timeout_ms);
    void serial_write(const uint8_t* buf, int len);

    static uint8_t checksum(uint8_t id, uint8_t length, uint8_t cmd,
                             const uint8_t* payload, uint8_t plen);
    static void pack_i16(uint8_t* buf, int16_t v);
    static void pack_u16(uint8_t* buf, uint16_t v);
    static void pack_u32(uint8_t* buf, uint32_t v);
};
