// debug_pitch_only.cpp
// Captures pairs where ONLY pitch changes (base fixed), checks A/B alignment.
// If only pitch rotations show misalignment, the Ry(-pitch) sign is wrong.
#include "../camera/orbbec_camera.hpp"
#include "../nexarm.hpp"
#include "../nexarm_config.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <iostream>
#include <thread>
#include <chrono>

static const int  BOARD_COLS=14,BOARD_ROWS=9;
static const float SQUARE_MM=20.f,MARKER_MM=15.f;
static const int  DICT_ID=cv::aruco::DICT_5X5_100;
static const int  CAM_W=1920,CAM_H=1080;
static const double DEG_PER_STEP=360.0/4096.0;
static const int  SERVO_CENTER=2048;

static cv::Matx33d Rz(double d){double r=d*3.14159265/180;return{cos(r),-sin(r),0,sin(r),cos(r),0,0,0,1};}
static cv::Matx33d Ry(double d){double r=d*3.14159265/180;return{cos(r),0,sin(r),0,1,0,-sin(r),0,cos(r)};}
static cv::Matx33d Rx(double d){double r=d*3.14159265/180;return{1,0,0,0,cos(r),-sin(r),0,sin(r),cos(r)};}

// Test different sign conventions for pitch using Rx (camera pitch axis is X)
static cv::Mat make_T(const NexArmCoords& c, int pitch_sign){
    double j1=+(c.servo[0]-SERVO_CENTER)*DEG_PER_STEP;
    cv::Matx33d R=Rz(j1)*Rx(pitch_sign*c.pitch);
    cv::Mat T=cv::Mat::eye(4,4,CV_64F);
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)T.at<double>(i,j)=R(i,j);
    T.at<double>(0,3)=c.x;T.at<double>(1,3)=c.y;T.at<double>(2,3)=c.z;
    return T;
}

static cv::Mat board_pose(cv::Mat& frame,cv::aruco::CharucoDetector& det,
                          cv::aruco::CharucoBoard& board,cv::Mat& K,cv::Mat& dist){
    cv::Mat gray;cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;std::vector<int> ids;
    try{det.detectBoard(gray,corners,ids);}catch(...){}
    if(corners.size()<6)return{};
    std::vector<cv::Point3f> obj;
    for(int id:ids)obj.push_back(board.getChessboardCorners()[id]);
    cv::Mat rv,tv;cv::solvePnP(obj,corners,K,dist,rv,tv);
    cv::Mat R;cv::Rodrigues(rv,R);
    cv::Mat T=cv::Mat::eye(4,4,CV_64F);
    R.convertTo(T(cv::Rect(0,0,3,3)),CV_64F);
    tv.convertTo(T(cv::Rect(3,0,1,3)),CV_64F);
    return T;
}

static double rot_angle(const cv::Mat& T){
    cv::Mat R=T(cv::Rect(0,0,3,3));
    double tr=(R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2)-1)/2;
    tr=(std::max)(-1.0,(std::min)(1.0,tr));
    return acos(tr)*180/3.14159265;
}

static double axis_dot(const cv::Mat& A,const cv::Mat& B){
    auto ax=[](const cv::Mat& T)->cv::Vec3d{
        cv::Mat R=T(cv::Rect(0,0,3,3));
        return{R.at<double>(2,1)-R.at<double>(1,2),
               R.at<double>(0,2)-R.at<double>(2,0),
               R.at<double>(1,0)-R.at<double>(0,1)};
    };
    auto a=ax(A),b=ax(B);
    double na=cv::norm(a),nb=cv::norm(b);
    if(na<1e-6||nb<1e-6)return 0;
    return a.dot(b)/(na*nb);
}

int main(int argc,char* argv[]){
    if(argc<2){std::cerr<<"Usage: debug_pitch_only.exe <COMx>\n";return 1;}
    auto cfg=NexArmConfig::auto_load();
    cv::Mat K=(cv::Mat_<double>(3,3)<<cfg.cam_fx,0,cfg.cam_cx,0,cfg.cam_fy,cfg.cam_cy,0,0,1);
    cv::Mat dist=cv::Mat::zeros(5,1,CV_64F);
    auto dict=cv::aruco::getPredefinedDictionary(DICT_ID);
    auto board=cv::aruco::CharucoBoard({BOARD_COLS,BOARD_ROWS},SQUARE_MM,MARKER_MM,dict);
    cv::aruco::CharucoDetector detector(board);
    OrbbecCamera cam(CAM_W,CAM_H,30);cam.open();cam.warmup(5);
    NexArmClient arm(argv[1]);arm.open();

    struct P{cv::Mat Tb;NexArmCoords c;};
    std::vector<P> poses;

    std::cout<<"KEEP BASE FIXED, only change pitch. SPACE=capture ESC=quit\n";

    while(true){
        auto frame=cam.capture();
        auto Tb=board_pose(frame,detector,board,K,dist);
        cv::Mat disp;cv::resize(frame,disp,{1280,720});
        cv::putText(disp,Tb.empty()?"No board":"DETECTED SPACE=capture",
            {10,30},cv::FONT_HERSHEY_SIMPLEX,0.8,Tb.empty()?cv::Scalar(0,80,220):cv::Scalar(0,220,0),2);
        cv::putText(disp,"Poses: "+std::to_string(poses.size()),
            {10,65},cv::FONT_HERSHEY_SIMPLEX,0.8,{200,200,0},2);
        cv::imshow("Pitch-only test",disp);
        int key=cv::waitKey(1)&0xFF;
        if(key==27)break;
        if(key==32&&!Tb.empty()){
            arm.flush_rx();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            auto c=arm.get_full_coords(2000);
            if(!c.has_full){std::cout<<"No full coords\n";continue;}
            poses.push_back({Tb,c});
            std::cout<<"Pose "<<poses.size()<<": j1="<<(int)(-(c.servo[0]-SERVO_CENTER)*DEG_PER_STEP)
                     <<"  pitch="<<(int)c.pitch<<"  x="<<c.x<<"\n";

            if(poses.size()>=2){
                auto& p0=poses[poses.size()-2];
                auto& p1=poses[poses.size()-1];
                cv::Mat B=p0.Tb.inv()*p1.Tb;
                double angB=rot_angle(B);
                // Extract rotation axis of B
                cv::Mat RB=B(cv::Rect(0,0,3,3));
                cv::Vec3d axB(RB.at<double>(2,1)-RB.at<double>(1,2),
                              RB.at<double>(0,2)-RB.at<double>(2,0),
                              RB.at<double>(1,0)-RB.at<double>(0,1));
                double nb=cv::norm(axB); if(nb>1e-6) axB/=nb;
                std::cout<<"  B rot="<<angB<<"deg  axis=["<<axB[0]<<","<<axB[1]<<","<<axB[2]<<"]\n";
                for(int ps:{1,-1}){
                    cv::Mat T0=make_T(p0.c,ps).inv();
                    cv::Mat T1=make_T(p1.c,ps).inv();
                    cv::Mat A=T0.inv()*T1;
                    double angA=rot_angle(A);
                    double dot=axis_dot(A,B);
                    std::cout<<"  pitch_sign="<<(ps>0?"+":"-")
                             <<"  A rot="<<angA<<"deg  dot="<<dot
                             <<(dot>0.9?" ALIGNED":dot>0?" partial":" MISALIGNED")<<"\n";
                }
            }
        }
    }
    cam.close();arm.close();
    return 0;
}
