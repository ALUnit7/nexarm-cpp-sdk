// Example 2: Move to home position, then incremental moves
// Usage: ex02_move.exe COM10
#include "../nexarm.hpp"
#include <iostream>
#include <thread>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    if (argc < 2) { std::cerr << "Usage: ex02_move.exe <COMx>\n"; return 1; }

    NexArmClient arm(argv[1]);
    arm.open();
    arm.set_buzzer(80, 0, 1, 2000);  // beep on start

    std::cout << "Moving to home (200,0,200,pitch=0)...\n";
    arm.set_pose(200, 0, 200, 0, 0, 0, 1500);
    std::this_thread::sleep_for(1800ms);

    auto read_pos = [&](const char* label) {
        arm.flush_rx();
        std::this_thread::sleep_for(200ms);
        auto c = arm.get_full_coords();
        std::cout << label << " x=" << c.x << " y=" << c.y
                  << " z=" << c.z << " pitch=" << c.pitch << "\n";
    };

    read_pos("Home:    ");

    std::cout << "dz -50mm...\n";
    arm.move_increment(0, 0, -50, 0, 0, 0, 800);
    std::this_thread::sleep_for(1000ms);
    read_pos("After dz:");

    std::cout << "dpitch -30deg...\n";
    arm.move_increment(0, 0, 0, -30, 0, 0, 800);
    std::this_thread::sleep_for(1000ms);
    read_pos("After dp:");

    std::cout << "Returning home...\n";
    arm.set_pose(200, 0, 200, 0, 0, 0, 1500);
    std::this_thread::sleep_for(1800ms);

    arm.close();
    return 0;
}
