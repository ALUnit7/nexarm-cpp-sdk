// calibrate_offset.cpp
// Measure T_cam2gripper offset by comparing:
//   visual estimate (camera sees red square) vs actual gripper position (manually moved)
// S=save visual estimate, R=record actual gripper pos, ESC=quit & print offsets
// Usage: calibrate_offset.exe COM11
#include "../camera/orbbec_camera.hpp"
#include "../nexarm.hpp"
#include "../nexarm_config.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

static const int  CAM_W=1920,CAM_H=1080;
static const double DEG=360.0/4096.0,SERVO_C=2048;
static const float SQUARE_MM=40.f;

static cv::Mat gripper2base(const NexArmCoords& c){
    double j1=-(c.servo[0]-SERVO_C)*DEG;
    double r=j1*3.14159265/180,p=-c.pitch*3.14159265/180;
    double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p);
    cv::Mat T=(cv::Mat_<double>(4,4)<<cr*cp,-sr,cr*sp,c.x,sr*cp,cr,sr*sp,c.y,-sp,0,cp,c.z,0,0,0,1);
    return T;
}

int main(int argc,char* argv[]){
    if(argc<2){std::cerr<<"Usage: calibrate_offset.exe <COMx>\n";return 1;}
    auto cfg=NexArmConfig::auto_load();
    cv::Mat K=(cv::Mat_<double>(3,3)<<cfg.cam_fx,0,cfg.cam_cx,0,cfg.cam_fy,cfg.cam_cy,0,0,1);
    cv::Mat dist=(cv::Mat_<double>(5,1)<<cfg.cam_dist[0],cfg.cam_dist[1],cfg.cam_dist[2],cfg.cam_dist[3],cfg.cam_dist[4]);
    cv::Mat Tc2g=cv::Mat::eye(4,4,CV_64F);
    for(int i=0;i<4;i++)for(int j=0;j<4;j++)Tc2g.at<double>(i,j)=cfg.hand_eye[i*4+j];

    OrbbecCamera cam(CAM_W,CAM_H,30); cam.open(); cam.warmup(5);
    NexArmClient  arm(argv[1]);        arm.open();

    struct Pair { cv::Vec3d visual, actual; };
    std::vector<Pair> pairs;
    cv::Vec3d last_visual(0,0,0); bool has_visual=false;

    std::cout<<"S=save visual estimate  R=record actual gripper pos  ESC=quit\n\n";

    while(true){
        auto frame=cam.capture();
        cv::Mat hsv; cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        cv::Mat m1,m2,mask;
        cv::inRange(hsv,cv::Scalar(0,120,80),cv::Scalar(10,255,255),m1);
        cv::inRange(hsv,cv::Scalar(160,120,80),cv::Scalar(180,255,255),m2);
        mask=m1|m2;
        cv::morphologyEx(mask,mask,cv::MORPH_OPEN,cv::Mat::ones(5,5,CV_8U));
        cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::Mat::ones(15,15,CV_8U));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

        cv::Mat disp; cv::resize(frame,disp,{1280,720});
        float sx=1280.f/CAM_W,sy=720.f/CAM_H;
        cv::RotatedRect best; double best_area=0; bool found=false;
        for(auto& c:contours){
            double a=cv::contourArea(c); if(a<500)continue;
            auto rr=cv::minAreaRect(c);
            double ratio=(std::max)(rr.size.width,rr.size.height)/((std::min)(rr.size.width,rr.size.height)+1e-6);
            if(ratio<1.8&&a>best_area){best_area=a;best=rr;found=true;}
        }
        if(found){
            cv::Point2f pts[4]; best.points(pts);
            for(int i=0;i<4;i++) cv::line(disp,{(int)(pts[i].x*sx),(int)(pts[i].y*sy)},{(int)(pts[(i+1)%4].x*sx),(int)(pts[(i+1)%4].y*sy)},{0,255,0},2);
            cv::putText(disp,"DETECTED  S=save visual",{10,35},cv::FONT_HERSHEY_SIMPLEX,0.8,{0,220,0},2);
        } else {
            cv::putText(disp,"No red square",{10,35},cv::FONT_HERSHEY_SIMPLEX,0.8,{0,80,220},2);
        }
        if(has_visual) cv::putText(disp,"Visual saved - move gripper, press R",{10,75},cv::FONT_HERSHEY_SIMPLEX,0.7,{200,200,0},2);
        cv::putText(disp,"Pairs: "+std::to_string(pairs.size()),{10,115},cv::FONT_HERSHEY_SIMPLEX,0.7,{180,180,180},1);
        cv::imshow("Calibrate Offset",disp);
        int key=cv::waitKey(1)&0xFF;
        if(key==27) break;

        if((key=='s'||key=='S')&&found){
            arm.flush_rx(); std::this_thread::sleep_for(std::chrono::milliseconds(300));
            auto c=arm.get_full_coords(2000);
            if(!c.has_full){std::cout<<"No arm coords\n";continue;}
            float px=(std::max)(best.size.width,best.size.height);
            float d=cfg.cam_fx*SQUARE_MM/px;
            cv::Mat pc=(cv::Mat_<double>(4,1)<<
                (best.center.x-cfg.cam_cx)/cfg.cam_fx*d,
                (best.center.y-cfg.cam_cy)/cfg.cam_fy*d,(double)d,1.0);
            cv::Mat pb=gripper2base(c)*Tc2g*pc;
            last_visual=cv::Vec3d(pb.at<double>(0),pb.at<double>(1),pb.at<double>(2));
            has_visual=true;
            std::cout<<"Visual estimate: ["<<(int)last_visual[0]<<","<<(int)last_visual[1]<<","<<(int)last_visual[2]<<"]\n";
            std::cout<<"Now manually move gripper to grasp position, then press R\n";
        }

        if((key=='r'||key=='R')&&has_visual){
            arm.flush_rx(); std::this_thread::sleep_for(std::chrono::milliseconds(300));
            auto c=arm.get_full_coords(2000);
            if(!c.has_full){std::cout<<"No arm coords\n";continue;}
            cv::Vec3d actual(c.x,c.y,c.z);
            cv::Vec3d offset=actual-last_visual;
            pairs.push_back({last_visual,actual});
            has_visual=false;
            std::cout<<"Actual gripper: ["<<c.x<<","<<c.y<<","<<c.z<<"]\n";
            std::cout<<"Offset (actual-visual): ["<<(int)offset[0]<<","<<(int)offset[1]<<","<<(int)offset[2]<<"]\n\n";
        }
    }

    cam.close(); arm.close(); cv::destroyAllWindows();

    if(pairs.size()>=2){
        cv::Vec3d mean_offset(0,0,0);
        for(auto& p:pairs) mean_offset+=p.actual-p.visual;
        mean_offset/=(double)pairs.size();
        std::cout<<"\n=== Results ("<<pairs.size()<<" pairs) ===\n";
        std::cout<<"Mean offset (actual - visual): ["<<(int)mean_offset[0]<<","<<(int)mean_offset[1]<<","<<(int)mean_offset[2]<<"] mm\n";
        std::cout<<"Add this to T_cam2gripper translation in nexarm_config.yaml\n";
        std::cout<<"Current t=[115.26,0,106.12] -> new t=["<<115.26+mean_offset[0]<<","<<mean_offset[1]<<","<<106.12+mean_offset[2]<<"]\n";
    }
    return 0;
}
