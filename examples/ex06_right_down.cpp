// ex06_right_down.cpp - slowly move arm to right side, gripper pointing down
// Usage: ex06_right_down.exe COM11
#include "../nexarm.hpp"
#include <iostream>
#include <thread>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    if (argc < 2) { std::cerr << "Usage: ex06_right_down.exe <COMx>\n"; return 1; }

    NexArmClient arm(argv[1]);
    arm.open();

    // Read current position
    arm.flush_rx();
    std::this_thread::sleep_for(300ms);
    auto c = arm.get_full_coords(500);
    std::cout << "Current: x=" << c.x << " y=" << c.y << " z=" << c.z
              << " pitch=" << c.pitch << "\n";

    // Step 1: raise Z first (safety)
    std::cout << "Step 1: Raising to safe height...\n";
    arm.set_pose(c.x, c.y, 250, c.pitch, 0, 0, 2000);
    std::this_thread::sleep_for(2200ms);

    // Step 2: slowly move to right side with pitch=-90 (gripper down)
    // Closer to base = easier to achieve pitch=-90
    std::cout << "Step 2: Moving to right side (y=-150), gripper down...\n";
    arm.set_pose(150, -150, 200, -90, 0, 0, 4000);
    std::this_thread::sleep_for(4500ms);

    arm.flush_rx(); std::this_thread::sleep_for(300ms);
    auto c2 = arm.get_full_coords(500);
    std::cout << "Arrived: x=" << c2.x << " y=" << c2.y << " z=" << c2.z
              << " pitch=" << c2.pitch << "\n";

    arm.close();
    return 0;
}
