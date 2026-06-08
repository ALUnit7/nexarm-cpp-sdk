// calibrate_intrinsic.cpp — ChArUco board support
#include "../camera/orbbec_camera.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <iostream>
#include <fstream>
#include <vector>

static const int   BOARD_COLS  = 14;
static const int   BOARD_ROWS  = 9;
static const float SQUARE_MM   = 20.f;
static const float MARKER_MM   = 15.f;
static const int   DICT_ID     = cv::aruco::DICT_5X5_100;
static const int   TARGET      = 30;
static const int   CAM_W       = 1920;
static const int   CAM_H       = 1080;

struct Coverage {
    int  count = 0;
    int  zones = 0;
    int  tilts = 0;
    bool dist  = false;
    std::string hint;
};

static Coverage analyse(const std::vector<std::vector<cv::Point2f>>& pts) {
    Coverage c; c.count = (int)pts.size();
    if (pts.empty()) { c.hint = "Show board to camera and press SPACE"; return c; }
    std::set<int> cells, tilts;
    float mina=1e9, maxa=0;
    for (auto& p : pts) {
        cv::Rect bb = cv::boundingRect(p);
        float cx=0,cy=0; for(auto& q:p){cx+=q.x;cy+=q.y;} cx/=p.size();cy/=p.size();
        cells.insert((int)(cx/CAM_W*3)*3+(int)(cy/CAM_H*3));
        tilts.insert((int)((float)bb.width/bb.height*5));
        float a=(float)(bb.width*bb.height); mina=(std::min)(mina,a); maxa=(std::max)(maxa,a);
    }
    c.zones = (int)cells.size();
    c.tilts = (int)tilts.size();
    c.dist  = maxa/mina > 2.f;
    if (c.zones < 6)      c.hint = "Move board to different screen areas";
    else if (c.tilts < 5) c.hint = "Tilt board at more angles";
    else if (!c.dist)     c.hint = "Vary distance: closer and farther";
    else if (c.count < TARGET) c.hint = "Good! Keep capturing.";
    else                  c.hint = "Ready. Press ESC to compute.";
    return c;
}

static void bar(cv::Mat& img, int x, int y, int w, float f,
                const std::string& lbl, bool ok) {
    cv::rectangle(img,{x,y},{x+w,y+18},{50,50,50},-1);
    int fi=(std::min)((int)(f*w),w);
    if(fi>0) cv::rectangle(img,{x,y},{x+fi,y+18},ok?cv::Scalar(0,200,80):cv::Scalar(0,140,220),-1);
    cv::rectangle(img,{x,y},{x+w,y+18},{150,150,150},1);
    cv::putText(img,lbl,{x+w+8,y+14},cv::FONT_HERSHEY_SIMPLEX,0.48,{210,210,210},1);
}

int main() {
    auto dict  = cv::aruco::getPredefinedDictionary(DICT_ID);
    auto board = cv::aruco::CharucoBoard({BOARD_COLS, BOARD_ROWS}, SQUARE_MM, MARKER_MM, dict);
    cv::aruco::CharucoDetector detector(board);

    std::vector<std::vector<cv::Point2f>> all_charuco;
    std::vector<std::vector<int>>         all_ids;

    OrbbecCamera cam(CAM_W, CAM_H, 30);
    cam.open(); cam.warmup(5);
    std::cout << "ChArUco " << BOARD_COLS << "x" << BOARD_ROWS
              << "  SPACE=capture  ESC=compute\n";

    while (true) {
        auto frame = cam.capture();
        cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        try { detector.detectBoard(gray, corners, ids); } catch(...) {}

        cv::Mat disp; cv::resize(frame, disp, {1280,720});
        float sx=1280.f/CAM_W, sy=720.f/CAM_H;

        bool found = !corners.empty() && (int)corners.size() >= 6;
        if (found) {
            std::vector<cv::Point2f> sc;
            for (auto& p : corners) sc.push_back({p.x*sx, p.y*sy});
            for (auto& p : sc)
                cv::circle(disp, p, 4, {0,255,0}, -1);
            cv::putText(disp, "corners=" + std::to_string(corners.size()),
                        {10,60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,220,0}, 2);
        }

        cv::rectangle(disp, {0,0}, {680,130}, {30,30,30}, -1);
        Coverage cov = analyse(all_charuco);
        bar(disp,10,  8,300,(float)cov.count/TARGET,
            "Captured: "+std::to_string(cov.count)+"/"+std::to_string(TARGET),cov.count>=TARGET);
        bar(disp,10, 33,300,cov.zones/9.f,
            "Frame zones: "+std::to_string(cov.zones)+"/9",cov.zones>=6);
        bar(disp,10, 58,300,cov.tilts/8.f,
            "Tilt variety: "+std::to_string(cov.tilts)+"/8",cov.tilts>=5);
        bar(disp,10, 83,300,cov.dist?1.f:0.3f,
            std::string("Distance variety: ")+(cov.dist?"OK":"need more"),cov.dist);
        cv::putText(disp, found?"DETECTED  SPACE=capture":"No board",
                    {10,118}, cv::FONT_HERSHEY_SIMPLEX,0.55,
                    found?cv::Scalar(0,220,0):cv::Scalar(0,80,220),1);
        cv::putText(disp, cov.hint, {340,30},
                    cv::FONT_HERSHEY_SIMPLEX,0.55,{200,200,0},1);

        cv::imshow("Intrinsic Calibration", disp);
        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;
        if (key == 32 && found) {
            all_charuco.push_back(corners);
            all_ids.push_back(ids);
            std::cout << "Captured " << all_charuco.size() << "/" << TARGET << "\n";
        }
    }

    cam.close(); cv::destroyAllWindows();
    if ((int)all_charuco.size() < 10) {
        std::cerr << "Not enough images.\n"; return 1;
    }

    cv::Mat K = cv::Mat::eye(3,3,CV_64F), dist = cv::Mat::zeros(5,1,CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::aruco::calibrateCameraCharuco(
        all_ids, all_charuco, board, {CAM_W, CAM_H}, K, dist, rvecs, tvecs);

    std::cout << "Error: " << rms << " px  ("
              << (rms<0.5?"EXCELLENT":rms<1.0?"GOOD":rms<2.0?"ACCEPTABLE":"POOR") << ")\n";
    std::cout << "fx=" << K.at<double>(0,0) << " fy=" << K.at<double>(1,1)
              << " cx=" << K.at<double>(0,2) << " cy=" << K.at<double>(1,2) << "\n";

    std::ofstream f("intrinsics.json");
    f << std::fixed << "{\n"
      << "  \"width\": " << CAM_W << ",\n"
      << "  \"height\": " << CAM_H << ",\n"
      << "  \"fx\": " << K.at<double>(0,0) << ",\n"
      << "  \"fy\": " << K.at<double>(1,1) << ",\n"
      << "  \"cx\": " << K.at<double>(0,2) << ",\n"
      << "  \"cy\": " << K.at<double>(1,2) << ",\n"
      << "  \"dist\": [";
    for (int i=0;i<dist.rows*dist.cols;i++){if(i)f<<",";f<<dist.at<double>(i);}
    f << "],\n  \"reprojection_error\": " << rms << "\n}\n";
    std::cout << "Saved: intrinsics.json\n";
    return 0;
}
