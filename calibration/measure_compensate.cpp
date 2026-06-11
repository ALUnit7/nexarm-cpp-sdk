// measure_compensate.cpp
// Point camera at ChArUco board, compute board origin in robot base frame.
// SPACE=compute & display, M=move arm to detected point, ESC=quit
// Usage: measure_compensate.exe COM11
#include "../camera/orbbec_camera.hpp"
#include "../nexarm.hpp"
#include "../nexarm_config.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <iostream>
#include <thread>
#include <chrono>

static const int  BOARD_COLS=14, BOARD_ROWS=9;
static const float SQUARE_MM=20.f, MARKER_MM=15.f;
static const int  DICT_ID=cv::aruco::DICT_5X5_100;
static const int  CAM_W=1920, CAM_H=1080;
static const double DEG=360.0/4096.0, SERVO_C=2048;

// Build T_gripper2base from current arm coords
// R = Rz(j1)*Ry(-pitch), t = [x,y,z]
static cv::Mat gripper2base(const NexArmCoords& c) {
    double j1 = -(c.servo[0] - SERVO_C) * DEG;
    double r  =  j1 * 3.14159265 / 180.0;   // base rotation
    double p  = -c.pitch * 3.14159265 / 180.0; // -pitch
    double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p);
    // Rz(j1)*Ry(-pitch):
    // col0=[cr*cp, sr*cp, -sp]  col1=[-sr, cr, 0]  col2=[cr*sp, sr*sp, cp]
    cv::Mat T = (cv::Mat_<double>(4,4) <<
        cr*cp, -sr, cr*sp, c.x,
        sr*cp,  cr, sr*sp, c.y,
          -sp,   0,    cp, c.z,
            0,   0,     0,   1);
    return T;
}

int main(int argc, char* argv[]) {
    if (argc < 2) { std::cerr << "Usage: measure_compensate.exe <COMx>\n"; return 1; }

    auto cfg = NexArmConfig::auto_load();
    if (!cfg.cam_intrinsics_valid()) {
        std::cerr << "No camera intrinsics in config.\n"; return 1;
    }
    if (!cfg.hand_eye_enabled) {
        std::cerr << "hand_eye.enabled=false in config. Set it to true with measured values.\n"; return 1;
    }

    cv::Mat K = (cv::Mat_<double>(3,3)
        << cfg.cam_fx, 0, cfg.cam_cx, 0, cfg.cam_fy, cfg.cam_cy, 0, 0, 1);
    cv::Mat dist = (cv::Mat_<double>(5,1)
        << cfg.cam_dist[0], cfg.cam_dist[1], cfg.cam_dist[2],
           cfg.cam_dist[3], cfg.cam_dist[4]);

    // T_cam2gripper from config (row-major 4x4)
    cv::Mat Tc2g = cv::Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            Tc2g.at<double>(i,j) = cfg.hand_eye[i*4+j];

    auto dict  = cv::aruco::getPredefinedDictionary(DICT_ID);
    auto board = cv::aruco::CharucoBoard({BOARD_COLS,BOARD_ROWS}, SQUARE_MM, MARKER_MM, dict);
    cv::aruco::CharucoDetector detector(board);

    OrbbecCamera cam(CAM_W, CAM_H, 30); cam.open(); cam.warmup(5);
    NexArmClient  arm(argv[1]);           arm.open();

    cv::Vec3d last_point_base(0,0,0);
    bool has_point = false;

    std::cout << "SPACE=detect & compute  M=move arm to point  ESC=quit\n";
    std::cout << "Adjust nexarm_config.yaml hand_eye.matrix based on errors.\n\n";

    while (true) {
        auto frame = cam.capture();
        cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners; std::vector<int> ids;
        try { detector.detectBoard(gray, corners, ids); } catch(...) {}
        bool found = !corners.empty() && (int)corners.size() >= 6;

        // Draw on display
        cv::Mat disp; cv::resize(frame, disp, {1280, 720});
        float sx=1280.f/CAM_W, sy=720.f/CAM_H;
        if (found) {
            for (auto& p : corners)
                cv::circle(disp, {(int)(p.x*sx),(int)(p.y*sy)}, 4, {0,255,0}, -1);
            cv::putText(disp, "DETECTED  SPACE=compute  M=move",
                {10,35}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,220,0}, 2);
        } else {
            cv::putText(disp, "No board", {10,35}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,80,220}, 2);
        }

        if (has_point) {
            std::string s = "Board origin in base: ["
                + std::to_string((int)last_point_base[0]) + ", "
                + std::to_string((int)last_point_base[1]) + ", "
                + std::to_string((int)last_point_base[2]) + "] mm";
            cv::putText(disp, s, {10,75}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {200,200,0}, 2);
        }

        cv::imshow("Measure & Compensate", disp);
        int key = cv::waitKey(1) & 0xFF;

        if (key == 27) break;

        if ((key == 32) && found) {
            // solvePnP: board origin in camera frame
            std::vector<cv::Point3f> obj;
            const auto& all = board.getChessboardCorners();
            for (int id : ids) if (id >= 0 && id < (int)all.size()) obj.push_back(all[id]);
            if (obj.size() != corners.size() || obj.size() < 6) continue;

            cv::Mat rv, tv;
            if (!cv::solvePnP(obj, corners, K, dist, rv, tv)) continue;

            // Board origin in camera frame = tvec
            cv::Mat p_cam = (cv::Mat_<double>(4,1) << tv.at<double>(0), tv.at<double>(1), tv.at<double>(2), 1.0);
            double dist_cam = cv::norm(tv);
            std::cout << "Board in camera frame: [" << (int)tv.at<double>(0) << ","
                      << (int)tv.at<double>(1) << "," << (int)tv.at<double>(2)
                      << "] dist=" << (int)dist_cam << "mm\n";

            // Read arm pose
            arm.flush_rx();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            auto c = arm.get_full_coords(2000);
            if (!c.has_full) { std::cout << "No arm coords\n"; continue; }

            // Transform: P_base = T_g2b * T_c2g * P_cam
            cv::Mat Tg2b = gripper2base(c);
            cv::Mat p_base = Tg2b * Tc2g * p_cam;

            last_point_base = cv::Vec3d(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
            has_point = true;

            std::cout << "Board origin in base: ["
                << (int)last_point_base[0] << ", "
                << (int)last_point_base[1] << ", "
                << (int)last_point_base[2] << "] mm"
                << "  (arm: x=" << c.x << " y=" << c.y << " z=" << c.z << " pitch=" << (int)c.pitch << ")\n";
        }

        if ((key == 'm' || key == 'M') && has_point) {
            double tx = last_point_base[0];
            double ty = last_point_base[1];
            double tz = last_point_base[2];
            std::cout << "Moving arm to ["<<(int)tx<<","<<(int)ty<<","<<(int)tz<<"] pitch=-90...\n";
            arm.set_pose((float)tx, (float)ty, (float)tz, -90.f, 0.f, 0.f, 2000);
            std::this_thread::sleep_for(std::chrono::milliseconds(2200));
            arm.flush_rx();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            auto c2 = arm.get_full_coords(2000);
            if (c2.has_full)
                std::cout << "Arrived at: x=" << c2.x << " y=" << c2.y << " z=" << c2.z << "\n";
            std::cout << "Observe offset and adjust nexarm_config.yaml hand_eye.matrix\n";
        }
    }

    cam.close(); arm.close(); cv::destroyAllWindows();
    return 0;
}
