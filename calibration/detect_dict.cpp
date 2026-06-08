// detect_dict.cpp - real-time ArUco dictionary detection
#include "../camera/orbbec_camera.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <iostream>

int main() {
    OrbbecCamera cam(1920, 1080, 30);
    cam.open();
    cam.warmup(5);

    const int dicts[] = {
        cv::aruco::DICT_4X4_50,  cv::aruco::DICT_4X4_100,
        cv::aruco::DICT_5X5_50,  cv::aruco::DICT_5X5_100,
        cv::aruco::DICT_6X6_50,  cv::aruco::DICT_6X6_100,  cv::aruco::DICT_6X6_250,
        cv::aruco::DICT_7X7_50,  cv::aruco::DICT_ARUCO_ORIGINAL
    };
    const char* names[] = {
        "4X4_50","4X4_100","5X5_50","5X5_100",
        "6X6_50","6X6_100","6X6_250","7X7_50","ARUCO_ORIGINAL"
    };
    const int N = sizeof(dicts)/sizeof(dicts[0]);

    // Pre-build detectors
    std::vector<cv::aruco::ArucoDetector> detectors;
    for (int i = 0; i < N; ++i)
        detectors.emplace_back(cv::aruco::getPredefinedDictionary(dicts[i]));

    std::cout << "Real-time detection. Press SPACE to print results. ESC=quit.\n";

    while (true) {
        auto frame = cam.capture();

        // Test all dicts, find best
        int best_i = -1, best_count = 0;
        for (int i = 0; i < N; ++i) {
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            detectors[i].detectMarkers(frame, corners, ids);
            if ((int)ids.size() > best_count) {
                best_count = ids.size();
                best_i = i;
            }
        }

        cv::Mat disp = frame.clone();
        if (best_i >= 0 && best_count > 0) {
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            detectors[best_i].detectMarkers(frame, corners, ids);
            cv::aruco::drawDetectedMarkers(disp, corners, ids);
            cv::putText(disp, std::string("Best: ") + names[best_i] +
                        "  markers=" + std::to_string(best_count),
                        {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,220,0}, 2);
        } else {
            cv::putText(disp, "No markers detected",
                        {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,0,220}, 2);
        }

        cv::imshow("Dict Detection", disp);
        int key = cv::waitKey(1);
        if (key == 27) break;
        if (key == 32) {
            std::cout << "\n--- Results ---\n";
            for (int i = 0; i < N; ++i) {
                std::vector<std::vector<cv::Point2f>> c; std::vector<int> ids;
                detectors[i].detectMarkers(frame, c, ids);
                std::cout << names[i] << ": " << ids.size() << " markers";
                if (!ids.empty()) {
                    std::cout << "  IDs:[";
                    for (int j=0;j<(int)ids.size()&&j<8;++j){if(j)std::cout<<",";std::cout<<ids[j];}
                    std::cout << "]";
                }
                std::cout << "\n";
            }
        }
    }
    cam.close();
    return 0;
}
