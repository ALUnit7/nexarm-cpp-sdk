// Example 5: OrbbecSDK camera - capture and display
// Press SPACE to save frame, ESC to quit.
#include "../camera/orbbec_camera.hpp"
#include "../nexarm_config.hpp"
#include <iostream>

int main() {
    auto cfg = NexArmConfig::auto_load();

    OrbbecCamera cam(1920, 1080, 30);
    cam.open();
    cam.warmup(10);

    // Prefer config intrinsics (SDK does not expose them for Gemini gen-1)
    if (cfg.cam_intrinsics_valid()) {
        std::cout << "Intrinsics (from config): fx=" << cfg.cam_fx
                  << " fy=" << cfg.cam_fy
                  << " cx=" << cfg.cam_cx
                  << " cy=" << cfg.cam_cy
                  << " size=" << cfg.cam_width << "x" << cfg.cam_height << "\n";
    } else {
        std::cout << "Intrinsics not configured (set camera_intrinsics in nexarm_config.yaml)\n";
    }

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
