// Example 4: Load config and use gripper with polarity support
// Usage: ex04_config.exe COM5
// Config is auto-loaded from exe directory or working directory.
#include "../nexarm.hpp"
#include "../nexarm_config.hpp"
#include <iostream>
#include <thread>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    if (argc < 2) { std::cerr << "Usage: ex04_config.exe <COMx>\n"; return 1; }

    auto cfg = NexArmConfig::auto_load();
    std::cout << "Gripper polarity : " << cfg.gripper_polarity << "\n";
    std::cout << "Gripper open val : " << cfg.gripper_open_value() << "\n";
    std::cout << "Gripper close val: " << cfg.gripper_close_value() << "\n";
    std::cout << "TCP offset       : x=" << cfg.tcp_x << " y=" << cfg.tcp_y << " z=" << cfg.tcp_z << "\n";

    NexArmClient arm(argv[1]);
    arm.open();

    arm.set_pose(200, 0, 200, -45, 0, 0, 1500);
    std::this_thread::sleep_for(1800ms);

    std::cout << "Opening gripper...\n";
    arm.set_pose(200, 0, 200, -45, 0, (float)cfg.gripper_open_value(), 500);
    std::this_thread::sleep_for(700ms);

    std::cout << "Closing gripper...\n";
    arm.set_pose(200, 0, 200, -45, 0, (float)cfg.gripper_close_value(), 500);
    std::this_thread::sleep_for(700ms);

    arm.close();
    return 0;
}
