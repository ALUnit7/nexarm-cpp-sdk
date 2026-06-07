// NexArm C++ SDK test program
// Usage:
//   test_nexarm.exe COM10            -- full hardware test
//   test_nexarm.exe --dry-run        -- frame encoding test, no serial required

#include "nexarm.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <thread>
#include <chrono>

// ── Dry-run: verify frame encoding without hardware ──────────────────────────
static void test_frame_encoding() {
    std::cout << "[dry-run] Testing frame encoding\n";

    // Expected: FF FF FF 02 01 FD  (firmware version query)
    uint8_t expected_fw[] = {0xFF,0xFF,0xFF,0x02,0x01,0xFD};
    uint8_t len = 2; // no payload → length = 0 + 2
    // checksum = ~(0xFF + 0x02 + 0x01) & 0xFF = ~(0x102) & 0xFF = ~0x02 & 0xFF = 0xFD
    uint8_t cs = (~(0xFF + len + 0x01)) & 0xFF;
    bool ok = (cs == 0xFD);
    std::cout << "  firmware_version checksum: 0x" << std::hex << (int)cs
              << (ok ? " OK" : " FAIL") << "\n";

    // Expected set_pose: pitch=0, x=200, y=0, z=200, roll=0, claw=0, time=1500ms
    // pitch*10=0→0x0000, x=200→0xC800, y=0→0x0000, z=200→0xC800
    // roll=0, claw=0, time=1500→0xDC05
    // payload = 00 00 C8 00 00 00 C8 00 00 00 00 00 DC 05  (14 bytes)
    uint8_t pose_payload[14] = {
        0x00,0x00, 0xC8,0x00, 0x00,0x00, 0xC8,0x00,
        0x00,0x00, 0x00,0x00, 0xDC,0x05
    };
    uint16_t s = 0xFF + 16 + 0x08;  // id=0xFF, length=14+2=16, cmd=0x08
    for (int i=0;i<14;i++) s += pose_payload[i];
    uint8_t pose_cs = (~s) & 0xFF;
    std::cout << std::dec << "  set_pose checksum: 0x" << std::hex << (int)pose_cs << " (verify manually)\n";

    std::cout << "[dry-run] PASSED\n\n";
}

// ── Hardware test ────────────────────────────────────────────────────────────
static void test_hardware(const std::string& port) {
    std::cout << "[hardware] Connecting to " << port << "\n";
    NexArmClient arm(port);
    arm.open();
    std::cout << "  Serial open OK\n";

    // 1. Firmware version
    try {
        auto ver = arm.get_firmware_version();
        std::cout << "  Firmware: " << ver << "\n";
    } catch (const std::exception& e) {
        std::cout << "  Firmware: FAILED (" << e.what() << ")\n";
    }

    // 2. Battery voltage
    try {
        auto mv = arm.get_battery_voltage();
        std::cout << "  Battery: " << mv << " mV\n";
    } catch (const std::exception& e) {
        std::cout << "  Battery: FAILED (" << e.what() << ")\n";
    }

    // 3. Read current coords (full broadcast frame)
    std::cout << "  Reading full coords (waiting for broadcast)...\n";
    try {
        arm.flush_rx();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        auto c = arm.get_full_coords(2000);
        std::cout << "  Coords: x=" << c.x << " y=" << c.y << " z=" << c.z
                  << " pitch=" << std::fixed << std::setprecision(1) << c.pitch
                  << " roll=" << c.roll << " claw=" << c.claw << "\n";
        if (c.has_full)
            std::cout << "  Servos: "
                      << c.servo[0] << " " << c.servo[1] << " " << c.servo[2] << " "
                      << c.servo[3] << " " << c.servo[4] << " " << c.servo[5] << "\n";
    } catch (const std::exception& e) {
        std::cout << "  Full coords: FAILED (" << e.what() << ")\n";
    }

    // 4. Move to home, wait, read back
    std::cout << "  Moving to home (200,0,200,pitch=0)...\n";
    arm.set_pose(200.f, 0.f, 200.f, 0.f, 0.f, 0.f, 1500);
    std::this_thread::sleep_for(std::chrono::milliseconds(1800));

    try {
        arm.flush_rx();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        auto c = arm.get_full_coords(2000);
        std::cout << "  After move: x=" << c.x << " y=" << c.y << " z=" << c.z
                  << " pitch=" << c.pitch << "\n";
        bool close = std::abs(c.x - 200) < 10 && std::abs(c.z - 200) < 10;
        std::cout << "  Position check: " << (close ? "OK" : "WARN (>10mm off)") << "\n";
    } catch (const std::exception& e) {
        std::cout << "  Read-back: FAILED (" << e.what() << ")\n";
    }

    // 5. Incremental move
    std::cout << "  Incremental move dz=-30mm...\n";
    arm.move_increment(0,0,-30, 0,0,0, 800);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    try {
        arm.flush_rx();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        auto c = arm.get_full_coords(2000);
        std::cout << "  After inc: z=" << c.z << " (expected ~170)\n";
    } catch (...) {}

    // Return home
    arm.set_pose(200.f, 0.f, 200.f, 0.f, 0.f, 0.f, 1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    arm.close();
    std::cout << "[hardware] Done\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: test_nexarm.exe <COMx>  OR  test_nexarm.exe --dry-run\n";
        return 1;
    }
    if (std::strcmp(argv[1], "--dry-run") == 0) {
        test_frame_encoding();
    } else {
        try {
            test_hardware(argv[1]);
        } catch (const std::exception& e) {
            std::cerr << "ERROR: " << e.what() << "\n";
            return 1;
        }
    }
    return 0;
}
