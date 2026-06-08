#pragma once
// OrbbecCamera: wrapper around OrbbecSDK v1 C++ API
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <stdexcept>

#include <libobsensor/ObSensor.hpp>

struct CameraIntrinsics {
    double fx, fy, cx, cy;
    int width, height;
    cv::Mat camera_matrix() const {
        return (cv::Mat_<double>(3,3) << fx,0,cx, 0,fy,cy, 0,0,1);
    }
    cv::Mat dist_coeffs() const { return cv::Mat::zeros(5,1,CV_64F); }
};

class OrbbecCamera {
public:
    explicit OrbbecCamera(int width = 1280, int height = 720, int fps = 30);
    ~OrbbecCamera();

    void open();
    void close();

    cv::Mat capture(int timeout_ms = 2000);
    void warmup(int frames = 10);
    CameraIntrinsics intrinsics() const;

private:
    int m_width, m_height, m_fps;
    std::shared_ptr<ob::Pipeline>           m_pipeline;
    std::shared_ptr<ob::VideoStreamProfile> m_profile;
    CameraIntrinsics                        m_intr = {};
};
