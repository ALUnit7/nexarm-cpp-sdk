// Example 5: OrbbecSDK camera - capture and display
// Usage: ex05_camera.exe
// Press SPACE to save frame, ESC to quit.
#include "../camera/orbbec_camera.hpp"
#include <iostream>

int main() {
    OrbbecCamera cam(1280, 720, 30);
    cam.open();

    auto intr = cam.intrinsics();
    std::cout << "Intrinsics: fx=" << intr.fx << " fy=" << intr.fy
              << " cx=" << intr.cx << " cy=" << intr.cy
              << " size=" << intr.width << "x" << intr.height << "\n";

    std::cout << "Warming up...\n";
    cam.warmup(10);
    std::cout << "Ready. SPACE=save ESC=quit\n";

    int saved = 0;
    while (true) {
        auto frame = cam.capture();
        cv::imshow("Orbbec", frame);
        int key = cv::waitKey(1);
        if (key == 27) break;
        if (key == 32) {
            std::string path = "frame_" + std::to_string(saved++) + ".png";
            cv::imwrite(path, frame);
            std::cout << "Saved: " << path << "\n";
        }
    }
    cam.close();
    return 0;
}
