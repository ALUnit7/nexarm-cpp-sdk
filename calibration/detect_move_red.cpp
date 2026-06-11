// detect_move_red.cpp
// Detect red square, estimate 3D position using known size (4cm),
// move arm directly above it with pitch=-90.
// SPACE=detect & move, ESC=quit
// Usage: detect_move_red.exe COM11
#include "../camera/orbbec_camera.hpp"
#include "../nexarm.hpp"
#include "../nexarm_config.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <thread>
#include <chrono>

static const int  CAM_W=1920, CAM_H=1080;
static const double DEG=360.0/4096.0, SERVO_C=2048;
static const float SQUARE_SIZE_MM = 40.f;  // 4cm red square
static const float HOVER_Z = 200.f;         // hover height mm

static cv::Mat gripper2base(const NexArmCoords& c) {
    double j1=-(c.servo[0]-SERVO_C)*DEG;
    double r=j1*3.14159265/180, p=-c.pitch*3.14159265/180;
    double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p);
    cv::Mat T=(cv::Mat_<double>(4,4)<<
        cr*cp,-sr,cr*sp,c.x,
        sr*cp, cr,sr*sp,c.y,
          -sp,  0,   cp,c.z,
            0,  0,    0,  1);
    return T;
}

int main(int argc, char* argv[]) {
    if(argc<2){std::cerr<<"Usage: detect_move_red.exe <COMx>\n";return 1;}
    auto cfg=NexArmConfig::auto_load();
    if(!cfg.cam_intrinsics_valid()||!cfg.hand_eye_enabled){
        std::cerr<<"Check config: cam_intrinsics and hand_eye must be set.\n";return 1;}
    cv::Mat K=(cv::Mat_<double>(3,3)
        <<cfg.cam_fx,0,cfg.cam_cx,0,cfg.cam_fy,cfg.cam_cy,0,0,1);
    cv::Mat dist=(cv::Mat_<double>(5,1)
        <<cfg.cam_dist[0],cfg.cam_dist[1],cfg.cam_dist[2],cfg.cam_dist[3],cfg.cam_dist[4]);
    cv::Mat Tc2g=cv::Mat::eye(4,4,CV_64F);
    for(int i=0;i<4;i++)for(int j=0;j<4;j++)Tc2g.at<double>(i,j)=cfg.hand_eye[i*4+j];

    OrbbecCamera cam(CAM_W,CAM_H,30); cam.open(); cam.warmup(5);
    NexArmClient  arm(argv[1]);        arm.open();

    std::cout<<"Point camera at red square (4cm). SPACE=detect&move  ESC=quit\n";

    while(true){
        auto frame=cam.capture();

        // Detect red square
        cv::Mat hsv; cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        cv::Mat mask1,mask2,mask;
        cv::inRange(hsv,cv::Scalar(0,120,80),cv::Scalar(10,255,255),mask1);
        cv::inRange(hsv,cv::Scalar(160,120,80),cv::Scalar(180,255,255),mask2);
        mask=mask1|mask2;
        cv::morphologyEx(mask,mask,cv::MORPH_OPEN,cv::Mat::ones(5,5,CV_8U));
        cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::Mat::ones(15,15,CV_8U));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

        cv::Mat disp; cv::resize(frame,disp,{1280,720});
        float sx=1280.f/CAM_W, sy=720.f/CAM_H;

        cv::RotatedRect best; double best_area=0; bool found=false;
        for(auto& c:contours){
            double area=cv::contourArea(c);
            if(area<500) continue;
            auto rr=cv::minAreaRect(c);
            double ratio=(std::max)(rr.size.width,rr.size.height)/
                         (std::min)(rr.size.width,rr.size.height);
            if(ratio<1.5 && area>best_area){best_area=area;best=rr;found=true;}
        }

        if(found){
            cv::Point2f pts[4]; best.points(pts);
            for(int i=0;i<4;i++)
                cv::line(disp,{(int)(pts[i].x*sx),(int)(pts[i].y*sy)},
                              {(int)(pts[(i+1)%4].x*sx),(int)(pts[(i+1)%4].y*sy)},{0,255,0},2);
            cv::circle(disp,{(int)(best.center.x*sx),(int)(best.center.y*sy)},6,{0,0,255},-1);
            cv::putText(disp,"RED SQUARE  SPACE=move above",{10,35},
                cv::FONT_HERSHEY_SIMPLEX,0.8,{0,220,0},2);
        } else {
            cv::putText(disp,"No red square",{10,35},cv::FONT_HERSHEY_SIMPLEX,0.8,{0,80,220},2);
        }
        cv::imshow("Red Square Detect",disp);
        int key=cv::waitKey(1)&0xFF;
        if(key==27) break;

        if(key==32 && found){
            // Estimate 3D: use known square size to get distance
            // Square pixel size -> distance via focal length
            float px_size=(std::max)(best.size.width,best.size.height);
            float dist_mm=cfg.cam_fx*SQUARE_SIZE_MM/px_size;
            float cx_norm=(best.center.x-cfg.cam_cx)/cfg.cam_fx;
            float cy_norm=(best.center.y-cfg.cam_cy)/cfg.cam_fy;
            cv::Mat p_cam=(cv::Mat_<double>(4,1)
                <<cx_norm*dist_mm,cy_norm*dist_mm,(double)dist_mm,1.0);

            arm.flush_rx();
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            auto c=arm.get_full_coords(2000);
            if(!c.has_full){std::cout<<"No arm coords\n";continue;}

            cv::Mat Tg2b=gripper2base(c);
            cv::Mat p_base=Tg2b*Tc2g*p_cam;
            double tx=p_base.at<double>(0), ty=p_base.at<double>(1);

            std::cout<<"Red square in base: x="<<(int)tx<<" y="<<(int)ty
                     <<" dist_cam="<<(int)dist_mm<<"mm\n";
            std::cout<<"Moving arm to x="<<(int)tx<<" y="<<(int)ty
                     <<" z="<<(int)HOVER_Z<<" pitch=-90...\n";

            arm.set_pose((float)tx,(float)ty,HOVER_Z,-90.f,0.f,0.f,2000);
            std::this_thread::sleep_for(std::chrono::milliseconds(2200));
            std::cout<<"Done. Check if gripper is aligned above square.\n";
        }
    }
    cam.close(); arm.close(); cv::destroyAllWindows();
    return 0;
}
