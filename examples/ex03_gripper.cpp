// Example 3: Gripper open/close test
// Usage: ex03_gripper.exe COM10
#include "../nexarm.hpp"
#include <iostream>
#include <thread>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    if (argc < 2) { std::cerr << "Usage: ex03_gripper.exe <COMx>\n"; return 1; }

    NexArmClient arm(argv[1]);
    arm.open();

    // Move to a convenient position first
    arm.set_pose(200, 0, 200, -45, 0, 0, 1500);
    std::this_thread::sleep_for(1800ms);

    std::cout << "Gripper open (claw=-80)...\n";
    arm.set_pose(200, 0, 200, -45, 0, -80, 500);
    std::this_thread::sleep_for(700ms);

    std::cout << "Gripper close (claw=0)...\n";
    arm.set_pose(200, 0, 200, -45, 0, 0, 500);
    std::this_thread::sleep_for(700ms);

    std::cout << "Gripper half-open (claw=-40)...\n";
    arm.set_pose(200, 0, 200, -45, 0, -40, 500);
    std::this_thread::sleep_for(700ms);

    std::cout << "Done. Reading claw position:\n";
    arm.flush_rx();
    std::this_thread::sleep_for(200ms);
    auto c = arm.get_full_coords();
    std::cout << "  claw=" << c.claw << "\n";

    arm.close();
    return 0;
}
