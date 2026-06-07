// Example 1: Connect and read arm status
// Usage: ex01_status.exe COM10
#include "../nexarm.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char* argv[]) {
    if (argc < 2) { std::cerr << "Usage: ex01_status.exe <COMx>\n"; return 1; }

    NexArmClient arm(argv[1]);
    arm.open();

    std::cout << "Firmware : " << arm.get_firmware_version() << "\n";
    std::cout << "Battery  : " << arm.get_battery_voltage() << " mV\n";

    arm.flush_rx();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    auto c = arm.get_full_coords();
    std::cout << "Position : x=" << c.x << " y=" << c.y << " z=" << c.z
              << " pitch=" << c.pitch << " roll=" << c.roll << " claw=" << c.claw << "\n";
    std::cout << "Servos   :";
    for (int i = 0; i < 6; ++i) std::cout << " " << c.servo[i];
    std::cout << "\n";

    arm.close();
    return 0;
}
