// NexArm C++ SDK implementation - Windows serial backend
// Linux port: replace open/close/serial_read/serial_write/flush_rx with termios equivalents

#include "nexarm.hpp"
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#endif

// ── Helpers ──────────────────────────────────────────────────────────────────
static inline int16_t read_i16_le(const uint8_t* p) {
    return (int16_t)(p[0] | (p[1] << 8));
}
void NexArmClient::pack_i16(uint8_t* b, int16_t v) { b[0]=v&0xFF; b[1]=(v>>8)&0xFF; }
void NexArmClient::pack_u16(uint8_t* b, uint16_t v){ b[0]=v&0xFF; b[1]=(v>>8)&0xFF; }
void NexArmClient::pack_u32(uint8_t* b, uint32_t v){ b[0]=v&0xFF; b[1]=(v>>8)&0xFF; b[2]=(v>>16)&0xFF; b[3]=(v>>24)&0xFF; }

uint8_t NexArmClient::checksum(uint8_t id, uint8_t length, uint8_t cmd,
                                const uint8_t* payload, uint8_t plen) {
    uint16_t s = id + length + cmd;
    for (int i = 0; i < plen; ++i) s += payload[i];
    return (~s) & 0xFF;
}

// ── Constructor / destructor ─────────────────────────────────────────────────
NexArmClient::NexArmClient(const std::string& port, uint32_t baud)
    : m_port(port), m_baud(baud), m_handle(INVALID_HANDLE_VALUE) {}

NexArmClient::~NexArmClient() { close(); }

// ── Serial backend (Windows) ─────────────────────────────────────────────────
// Linux port: replace this entire section with termios open/read/write/close
void NexArmClient::open() {
    if (m_handle != INVALID_HANDLE_VALUE) return;
#ifdef _WIN32
    std::string path = "\\\\.\\" + m_port;
    m_handle = CreateFileA(path.c_str(), GENERIC_READ|GENERIC_WRITE, 0,
                           nullptr, OPEN_EXISTING, 0, nullptr);
    if (m_handle == INVALID_HANDLE_VALUE)
        throw std::runtime_error("Cannot open " + m_port);

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(m_handle, &dcb);
    dcb.BaudRate = m_baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(m_handle, &dcb);

    COMMTIMEOUTS to = {};
    to.ReadIntervalTimeout         = 1;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.ReadTotalTimeoutConstant    = 1;   // 1ms per call; callers loop with deadline
    SetCommTimeouts(m_handle, &to);
#else
    m_handle = ::open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_handle < 0) throw std::runtime_error("Cannot open " + m_port);
    struct termios tty = {};
    tcgetattr(m_handle, &tty);
    cfsetspeed(&tty, B1000000);
    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    tcsetattr(m_handle, TCSANOW, &tty);
    fcntl(m_handle, F_SETFL, 0);  // blocking
#endif
}

void NexArmClient::close() {
    if (m_handle == INVALID_HANDLE_VALUE) return;
#ifdef _WIN32
    CloseHandle(m_handle);
#else
    ::close(m_handle);
#endif
    m_handle = INVALID_HANDLE_VALUE;
}

int NexArmClient::serial_read(uint8_t* buf, int len, int timeout_ms) {
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);
    int got = 0;
    while (got < len && std::chrono::steady_clock::now() < deadline) {
#ifdef _WIN32
        DWORD n = 0;
        ReadFile(m_handle, buf + got, len - got, &n, nullptr);
        got += (int)n;
        if (n == 0) std::this_thread::sleep_for(std::chrono::microseconds(500));
#else
        fd_set fs; FD_ZERO(&fs); FD_SET(m_handle, &fs);
        auto rem = deadline - std::chrono::steady_clock::now();
        long us = std::chrono::duration_cast<std::chrono::microseconds>(rem).count();
        if (us <= 0) break;
        struct timeval tv = { us / 1000000, (int)(us % 1000000) };
        if (select(m_handle+1, &fs, nullptr, nullptr, &tv) > 0) {
            ssize_t n = read(m_handle, buf + got, len - got);
            if (n > 0) got += (int)n;
        }
#endif
    }
    return got;
}

void NexArmClient::serial_write(const uint8_t* buf, int len) {
#ifdef _WIN32
    DWORD n;
    WriteFile(m_handle, buf, len, &n, nullptr);
#else
    write(m_handle, buf, len);
#endif
}

void NexArmClient::flush_rx() {
#ifdef _WIN32
    PurgeComm(m_handle, PURGE_RXCLEAR);
#else
    tcflush(m_handle, TCIFLUSH);
#endif
}

// ── Frame I/O ────────────────────────────────────────────────────────────────
void NexArmClient::send(uint8_t id, uint8_t cmd, const uint8_t* payload, uint8_t plen) {
    std::vector<uint8_t> frame(6 + plen);
    uint8_t length = plen + 2;
    frame[0] = 0xFF; frame[1] = 0xFF;
    frame[2] = id;
    frame[3] = length;
    frame[4] = cmd;
    if (plen && payload) memcpy(frame.data() + 5, payload, plen);
    frame[5 + plen] = checksum(id, length, cmd, payload ? payload : (const uint8_t*)"", plen);
    serial_write(frame.data(), 6 + plen);
}

NexArmPacket NexArmClient::read_packet(int timeout_ms) {
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);
    int state = 0;
    NexArmPacket pkt = {};

    while (std::chrono::steady_clock::now() < deadline) {
        auto rem = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        uint8_t b;
        if (serial_read(&b, 1, (int)std::max<long>(rem, 1)) != 1) continue;

        switch (state) {
        case 0: if (b == 0xFF) state = 1; break;
        case 1: state = (b == 0xFF) ? 2 : 0; break;
        case 2: pkt.id = b; state = 3; break;
        case 3:
            pkt.length = b;
            if (b < 2) { state = 0; break; }
            pkt.payload.clear();
            state = 4;
            break;
        case 4: pkt.cmd = b; state = (pkt.length > 2) ? 5 : 6; break;
        case 5:
            pkt.payload.push_back(b);
            if ((int)pkt.payload.size() == pkt.length - 2) state = 6;
            break;
        case 6: {
            uint8_t exp = checksum(pkt.id, pkt.length, pkt.cmd,
                                   pkt.payload.data(), (uint8_t)pkt.payload.size());
            if (b != exp) { state = 0; break; }
            return pkt;
        }
        }
    }
    throw std::runtime_error("read_packet: timeout");
}

NexArmPacket NexArmClient::request(uint8_t id, uint8_t cmd,
                                    const uint8_t* payload, uint8_t plen,
                                    uint8_t expect_cmd, uint8_t expect_id,
                                    int timeout_ms) {
    send(id, cmd, payload, plen);
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        auto rem = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        try {
            auto pkt = read_packet((int)std::max<long>(rem, 10));
            if (pkt.cmd == expect_cmd && pkt.id == expect_id) return pkt;
        } catch (...) { break; }
    }
    throw std::runtime_error("request: timeout waiting for cmd=0x"
                             + std::to_string(expect_cmd));
}

// ── High-level API ───────────────────────────────────────────────────────────
std::string NexArmClient::get_firmware_version(int timeout_ms) {
    auto pkt = request(NEXARM_SYSTEM_ID, NexArmCmd::FIRMWARE_VERSION,
                       nullptr, 0, NexArmCmd::FIRMWARE_VERSION, NEXARM_SYSTEM_ID, timeout_ms);
    if (pkt.payload.size() < 3) throw std::runtime_error("firmware version: short reply");
    return std::to_string(pkt.payload[0]) + "."
         + std::to_string(pkt.payload[1]) + "."
         + std::to_string(pkt.payload[2]);
}

uint16_t NexArmClient::get_battery_voltage(int timeout_ms) {
    auto pkt = request(NEXARM_SYSTEM_ID, NexArmCmd::BATTERY_LEVEL,
                       nullptr, 0, NexArmCmd::BATTERY_LEVEL, NEXARM_SYSTEM_ID, timeout_ms);
    if (pkt.payload.size() < 2) throw std::runtime_error("battery: short reply");
    return (uint16_t)(pkt.payload[0] | (pkt.payload[1] << 8));
}

void NexArmClient::set_pose(float x, float y, float z,
                             float pitch, float roll, float claw,
                             uint16_t duration_ms) {
    uint8_t buf[14];
    pack_i16(buf+0, (int16_t)std::round(pitch * 10.0f));
    pack_i16(buf+2, (int16_t)std::round(x));
    pack_i16(buf+4, (int16_t)std::round(y));
    pack_i16(buf+6, (int16_t)std::round(z));
    pack_i16(buf+8, (int16_t)std::round(roll));
    pack_i16(buf+10,(int16_t)std::round(claw));
    pack_u16(buf+12, duration_ms);
    send(NEXARM_SYSTEM_ID, NexArmCmd::COORDINATE_SET, buf, 14);
}

void NexArmClient::move_increment(float dx, float dy, float dz,
                                   float dpitch, float droll, float dclaw,
                                   uint16_t duration_ms) {
    uint8_t buf[14];
    pack_i16(buf+0, (int16_t)std::round(dx));
    pack_i16(buf+2, (int16_t)std::round(dy));
    pack_i16(buf+4, (int16_t)std::round(dz));
    pack_i16(buf+6, (int16_t)std::round(dpitch * 10.0f));
    pack_i16(buf+8, (int16_t)std::round(droll));
    pack_i16(buf+10,(int16_t)std::round(dclaw));
    pack_u16(buf+12, duration_ms);
    send(NEXARM_SYSTEM_ID, NexArmCmd::ARM_MOVE_INC, buf, 14);
}

static NexArmCoords parse_coords(const NexArmPacket& pkt) {
    NexArmCoords c = {};
    const auto& p = pkt.payload;
    if (p.size() < 6) throw std::runtime_error("coords reply too short");
    c.x = read_i16_le(p.data() + 0);
    c.y = read_i16_le(p.data() + 2);
    c.z = read_i16_le(p.data() + 4);
    if (p.size() >= 12) {
        c.pitch = read_i16_le(p.data() + 6) / 10.0f;
        c.roll  = read_i16_le(p.data() + 8);
        c.claw  = read_i16_le(p.data() + 10);
        c.has_full = true;
    }
    if (p.size() >= 24) {
        for (int i = 0; i < 6; ++i)
            c.servo[i] = read_i16_le(p.data() + 12 + i*2);
    }
    return c;
}

NexArmCoords NexArmClient::get_current_coords(int timeout_ms) {
    auto pkt = request(NEXARM_SYSTEM_ID, NexArmCmd::GET_CUR_COORDS,
                       nullptr, 0, NexArmCmd::GET_CUR_COORDS, NEXARM_SYSTEM_ID, timeout_ms);
    return parse_coords(pkt);
}

NexArmCoords NexArmClient::get_full_coords(int timeout_ms) {
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        auto rem = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        try {
            auto pkt = read_packet((int)std::max<long>(rem, 50));
            if (pkt.cmd == NexArmCmd::GET_CUR_COORDS && pkt.payload.size() >= 24)
                return parse_coords(pkt);
        } catch (...) { break; }
    }
    throw std::runtime_error("get_full_coords: timeout");
}

void NexArmClient::set_buzzer(uint32_t on_ms, uint32_t off_ms,
                               uint16_t times, uint16_t freq) {
    uint8_t buf[12];
    pack_u32(buf+0, on_ms);
    pack_u32(buf+4, off_ms);
    pack_u16(buf+8, times);
    pack_u16(buf+10, freq);
    send(NEXARM_SYSTEM_ID, NexArmCmd::BUZZER_SET, buf, 12);
}

void NexArmClient::servo_set_position(uint8_t servo_id, uint16_t position) {
    uint8_t buf[3];
    buf[0] = 0x2A;  // REG_POS
    pack_u16(buf+1, position);
    send(servo_id, 0x03, buf, 3);  // CMD_WRITE
}
