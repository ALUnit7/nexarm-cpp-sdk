// debug_ab.cpp - diagnose A and B matrices for hand-eye calibration
// Captures pairs of poses, prints A (robot motion) and B (board motion)
// A and B should have similar rotation angles for AX=XB to work.
#include "../camera/orbbec_camera.hpp"
#include "../nexarm.hpp"
#include "../nexarm_config.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <iostream>
#include <thread>
#include <chrono>

static const int  BOARD_COLS  = 14, BOARD_ROWS = 9;
static const float SQUARE_MM  = 20.f, MARKER_MM = 15.f;
static const int  DICT_ID     = cv::aruco::DICT_5X5_100;
static const int  CAM_W=1920, CAM_H=1080;
static const double DEG_PER_STEP = 360.0/4096.0;
static const int  SERVO_CENTER = 2048;

static cv::Matx33d Rz(double d){double r=d*3.14159265/180;return{cos(r),-sin(r),0,sin(r),cos(r),0,0,0,1};}
static cv::Matx33d Ry(double d){double r=d*3.14159265/180;return{cos(r),0,sin(r),0,1,0,-sin(r),0,cos(r)};}
static cv::Matx33d Rx(double d){double r=d*3.14159265/180;return{1,0,0,0,cos(r),-sin(r),0,sin(r),cos(r)};}

static cv::Mat make_T(const NexArmCoords& c){
    double j1=+(c.servo[0]-SERVO_CENTER)*DEG_PER_STEP;
    cv::Matx33d R=Rz(j1)*Rx(-c.pitch);
    cv::Mat T=cv::Mat::eye(4,4,CV_64F);
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)T.at<double>(i,j)=R(i,j);
    T.at<double>(0,3)=c.x;T.at<double>(1,3)=c.y;T.at<double>(2,3)=c.z;
    return T;
}

static cv::Mat board_pose(cv::Mat& frame, cv::aruco::CharucoDetector& det,
                          cv::aruco::CharucoBoard& board, cv::Mat& K, cv::Mat& dist){
    cv::Mat gray; cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners; std::vector<int> ids;
    det.detectBoard(gray,corners,ids);
    if(corners.size()<6) return {};
    std::vector<cv::Point3f> obj;
    for(int id:ids) obj.push_back(board.getChessboardCorners()[id]);
    cv::Mat rv,tv; cv::solvePnP(obj,corners,K,dist,rv,tv);
    cv::Mat R; cv::Rodrigues(rv,R);
    cv::Mat T=cv::Mat::eye(4,4,CV_64F);
    R.convertTo(T(cv::Rect(0,0,3,3)),CV_64F);
    tv.convertTo(T(cv::Rect(3,0,1,3)),CV_64F);
    return T;
}

static double rot_angle(const cv::Mat& T){
    cv::Mat R=T(cv::Rect(0,0,3,3));
    double trace=(R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2)-1)/2;
    trace=(std::max)(-1.0,(std::min)(1.0,trace));
    return std::acos(trace)*180/3.14159265;
}

int main(int argc,char* argv[]){
    if(argc<2){std::cerr<<"Usage: debug_ab.exe <COMx>\n";return 1;}
    auto cfg=NexArmConfig::auto_load();
    cv::Mat K=(cv::Mat_<double>(3,3)<<cfg.cam_fx,0,cfg.cam_cx,0,cfg.cam_fy,cfg.cam_cy,0,0,1);
    cv::Mat dist=cv::Mat::zeros(5,1,CV_64F);
    auto dict=cv::aruco::getPredefinedDictionary(DICT_ID);
    auto board=cv::aruco::CharucoBoard({BOARD_COLS,BOARD_ROWS},SQUARE_MM,MARKER_MM,dict);
    cv::aruco::CharucoDetector detector(board);

    OrbbecCamera cam(CAM_W,CAM_H,30); cam.open(); cam.warmup(5);
    NexArmClient arm(argv[1]); arm.open();

    std::cout<<"Capture pairs: move arm then press SPACE (need 2 captures for each A/B pair)\n";
    std::cout<<"ESC=quit\n";

    struct Pose{ cv::Mat T_robot, T_board; double j1,pitch,x,y,z; };
    std::vector<Pose> poses;

    while(true){
        auto frame=cam.capture();
        cv::Mat disp; cv::resize(frame,disp,{1280,720});
        cv::putText(disp,std::string("Poses: ")+std::to_string(poses.size())+
            "  SPACE=capture ESC=quit",{10,30},cv::FONT_HERSHEY_SIMPLEX,0.8,{0,220,0},2);
        cv::imshow("Debug A/B",disp);
        int key=cv::waitKey(1)&0xFF;
        if(key==27) break;
        if(key==32){
            auto Tb=board_pose(frame,detector,board,K,dist);
            if(Tb.empty()){std::cout<<"Board not detected, skip\n";continue;}
            arm.flush_rx();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            auto c=arm.get_full_coords(2000);
            if(!c.has_full){std::cout<<"No full coords\n";continue;}
            double j1=+(c.servo[0]-SERVO_CENTER)*DEG_PER_STEP;
            Pose p; p.T_robot=make_T(c); p.T_board=Tb;
            p.j1=j1;p.pitch=c.pitch;p.x=c.x;p.y=c.y;p.z=c.z;
            poses.push_back(p);
            std::cout<<"Pose "<<poses.size()<<": j1="<<(int)j1<<"  pitch="<<(int)c.pitch
                     <<"  x="<<c.x<<"  y="<<c.y<<"  z="<<c.z<<"\n";

            if(poses.size()>=2){
                int n=(int)poses.size();
                auto& p0=poses[n-2]; auto& p1=poses[n-1];
                // Use inv() for gripper2base convention (same as calibrateHandEye)
                cv::Mat A=p1.T_robot.inv()*p0.T_robot;  // calibrateHandEye: A = T_{i+1}^{-1} * T_i
                cv::Mat B=p0.T_board.inv()*p1.T_board;
                double angA=rot_angle(A), angB=rot_angle(B);
                double dxA=A.at<double>(0,3),dyA=A.at<double>(1,3),dzA=A.at<double>(2,3);
                double dxB=B.at<double>(0,3),dyB=B.at<double>(1,3),dzB=B.at<double>(2,3);
                std::cout<<"  A rot="<<angA<<"deg  t=["<<dxA<<","<<dyA<<","<<dzA<<"]\n";
                std::cout<<"  B rot="<<angB<<"deg  t=["<<dxB<<","<<dyB<<","<<dzB<<"]\n";
                std::cout<<"  Rotation match: "<<(std::abs(angA-angB)<5?"OK (diff<5deg)":"MISMATCH!")<<"\n";
                // Print rotation axes for diagnosis
                auto rot_axis = [](const cv::Mat& T) -> cv::Vec3d {
                    cv::Mat R = T(cv::Rect(0,0,3,3));
                    // axis from skew-symmetric part of R
                    return cv::Vec3d(
                        R.at<double>(2,1)-R.at<double>(1,2),
                        R.at<double>(0,2)-R.at<double>(2,0),
                        R.at<double>(1,0)-R.at<double>(0,1));
                };
                auto axA = rot_axis(A), axB = rot_axis(B);
                double na=cv::norm(axA), nb=cv::norm(axB);
                if(na>1e-6 && nb>1e-6){
                    axA/=na; axB/=nb;
                    double dot=axA.dot(axB);
                    std::cout<<"  Axis dot product: "<<dot
                             <<(std::abs(dot)>0.9?" (axes aligned)":" (axes MISALIGNED!)")<<"\n";
                }
            }
        }
    }
    cam.close(); arm.close();
    return 0;
}
