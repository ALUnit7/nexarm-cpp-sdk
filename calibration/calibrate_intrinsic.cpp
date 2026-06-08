// calibrate_intrinsic.cpp  — with real-time coverage guidance
#include "../camera/orbbec_camera.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <cmath>

static const cv::Size  PATTERN(11, 8);
static const float     SQUARE_MM = 20.f;
static const int       TARGET    = 30;
static const int       CAM_W     = 1920;
static const int       CAM_H     = 1080;

// ── Coverage analysis ────────────────────────────────────────────────────────
struct Coverage {
    int  count        = 0;
    int  area_cells   = 0;   // how many 3x3 grid cells covered
    int  tilt_buckets = 0;   // number of distinct tilt levels
    bool dist_varied  = false;

    std::string hint;
};

static Coverage analyse(const std::vector<std::vector<cv::Point2f>>& pts) {
    Coverage c;
    c.count = (int)pts.size();
    if (pts.empty()) return c;

    // 3x3 grid coverage
    std::set<int> cells;
    for (auto& corners : pts) {
        float cx = 0, cy = 0;
        for (auto& p : corners) { cx += p.x; cy += p.y; }
        cx /= corners.size(); cy /= corners.size();
        int col = (int)(cx / CAM_W * 3);
        int row = (int)(cy / CAM_H * 3);
        cells.insert(row * 3 + col);
    }
    c.area_cells = (int)cells.size();

    // Tilt: use bounding box aspect ratio as proxy
    std::set<int> tilts;
    for (auto& corners : pts) {
        cv::Rect bb = cv::boundingRect(corners);
        float ratio = (float)bb.width / bb.height;
        tilts.insert((int)(ratio * 5)); // bucket
    }
    c.tilt_buckets = (int)tilts.size();

    // Distance: spread of board areas
    float min_area = 1e9, max_area = 0;
    for (auto& corners : pts) {
        cv::Rect bb = cv::boundingRect(corners);
        float area = (float)(bb.width * bb.height);
        min_area = std::min(min_area, area);
        max_area = std::max(max_area, area);
    }
    c.dist_varied = (max_area / min_area) > 2.0f;

    // Hint
    if (c.area_cells < 7)
        c.hint = "Move board to different areas of the frame";
    else if (c.tilt_buckets < 6)
        c.hint = "Tilt the board at different angles";
    else if (!c.dist_varied)
        c.hint = "Vary distance: move closer then farther";
    else if (c.count < TARGET)
        c.hint = "Good coverage! Keep capturing diverse poses.";
    else
        c.hint = "Ready to compute. Press ESC.";

    return c;
}

// ── Draw progress bar ────────────────────────────────────────────────────────
static void bar(cv::Mat& img, int x, int y, int w,
                float fill, const std::string& label, bool ok) {
    cv::rectangle(img, {x, y}, {x+w, y+18}, {60,60,60}, -1);
    int filled = (int)(fill * w);
    cv::Scalar col = ok ? cv::Scalar(0,200,80) : cv::Scalar(0,140,220);
    if (filled > 0)
        cv::rectangle(img, {x, y}, {x+filled, y+18}, col, -1);
    cv::rectangle(img, {x, y}, {x+w, y+18}, {150,150,150}, 1);
    cv::putText(img, label, {x+w+10, y+14},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {220,220,220}, 1);
}

int main() {
    std::vector<cv::Point3f> objp;
    for (int r = 0; r < PATTERN.height; ++r)
        for (int c = 0; c < PATTERN.width; ++c)
            objp.emplace_back(c * SQUARE_MM, r * SQUARE_MM, 0.f);

    std::vector<std::vector<cv::Point3f>> obj_pts;
    std::vector<std::vector<cv::Point2f>> img_pts;

    const auto criteria = cv::TermCriteria(
        cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

    OrbbecCamera cam(CAM_W, CAM_H, 30);
    cam.open();
    cam.warmup(5);

    std::cout << "SPACE=capture  ESC=compute\n";

    while (true) {
        cv::Mat frame = cam.capture();
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, PATTERN, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        cv::Mat disp;
        cv::resize(frame, disp, {1280, 720});  // display at 720p for speed
        float sx = 1280.f/CAM_W, sy = 720.f/CAM_H;

        if (found) {
            cv::cornerSubPix(gray, corners, {11,11}, {-1,-1}, criteria);
            // Scale corners for display
            std::vector<cv::Point2f> sc;
            for (auto& p : corners) sc.push_back({p.x*sx, p.y*sy});
            cv::drawChessboardCorners(disp, PATTERN, sc, found);
        }

        // Coverage panel (dark background)
        cv::rectangle(disp, {0,0}, {680, 140}, {30,30,30}, -1);

        Coverage cov = analyse(img_pts);

        int n = cov.count;
        bar(disp, 10, 10, 300, (float)n/TARGET,
            "Captured: " + std::to_string(n) + "/" + std::to_string(TARGET), n>=TARGET);
        bar(disp, 10, 38, 300, cov.area_cells/9.f,
            "Frame coverage: " + std::to_string(cov.area_cells) + "/9 zones", cov.area_cells>=7);
        bar(disp, 10, 66, 300, cov.tilt_buckets/8.f,
            "Tilt diversity: " + std::to_string(cov.tilt_buckets) + "/8", cov.tilt_buckets>=6);
        bar(disp, 10, 94, 300, cov.dist_varied ? 1.f : 0.3f,
            std::string("Distance variety: ") + (cov.dist_varied ? "OK" : "need more"), cov.dist_varied);

        // Status / hint
        cv::Scalar sc_col = found ? cv::Scalar(0,220,0) : cv::Scalar(0,80,220);
        std::string status = found ? "DETECTED  SPACE=capture" : "No pattern";
        cv::putText(disp, status, {10, 128}, cv::FONT_HERSHEY_SIMPLEX, 0.6, sc_col, 1);
        cv::putText(disp, cov.hint, {690, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {200,200,0}, 1);

        cv::imshow("Intrinsic Calibration", disp);
        int key = cv::waitKey(1) & 0xFF;

        if (key == 27) break;
        if (key == 32 && found) {
            cv::cornerSubPix(gray, corners, {11,11}, {-1,-1}, criteria);
            obj_pts.push_back(objp);
            img_pts.push_back(corners);
            std::cout << "Captured " << obj_pts.size() << "/" << TARGET << "\n";
        }
    }

    cam.close();
    cv::destroyAllWindows();

    if ((int)obj_pts.size() < 10) {
        std::cerr << "Not enough images (" << obj_pts.size() << "), need at least 10.\n";
        return 1;
    }

    std::cout << "\nComputing from " << obj_pts.size() << " images...\n";
    cv::Mat K, dist;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(obj_pts, img_pts, {CAM_W, CAM_H}, K, dist, rvecs, tvecs);

    std::cout << "Reprojection error: " << rms << " px  ("
              << (rms < 0.5 ? "EXCELLENT" : rms < 1.0 ? "GOOD" : rms < 2.0 ? "ACCEPTABLE" : "POOR") << ")\n";
    std::cout << "fx=" << K.at<double>(0,0) << " fy=" << K.at<double>(1,1)
              << " cx=" << K.at<double>(0,2) << " cy=" << K.at<double>(1,2) << "\n";

    std::ofstream f("intrinsics.json");
    f << std::fixed;
    f << "{\n"
      << "  \"width\": "  << CAM_W << ",\n"
      << "  \"height\": " << CAM_H << ",\n"
      << "  \"fx\": " << K.at<double>(0,0) << ",\n"
      << "  \"fy\": " << K.at<double>(1,1) << ",\n"
      << "  \"cx\": " << K.at<double>(0,2) << ",\n"
      << "  \"cy\": " << K.at<double>(1,2) << ",\n"
      << "  \"dist\": [";
    for (int i = 0; i < dist.cols * dist.rows; ++i) {
        if (i) f << ", ";
        f << dist.at<double>(i);
    }
    f << "],\n"
      << "  \"reprojection_error\": " << rms << "\n"
      << "}\n";
    f.close();
    std::cout << "Saved: intrinsics.json\n";
    return 0;
}
