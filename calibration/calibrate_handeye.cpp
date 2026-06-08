// calibrate_handeye.cpp — ChArUco board, coverage guidance
#include "../camera/orbbec_camera.hpp"
#include "../nexarm.hpp"
#include "../nexarm_config.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <set>
#include <thread>
#include <chrono>

static const int   BOARD_COLS   = 14;
static const int   BOARD_ROWS   = 9;
static const float SQUARE_MM    = 20.f;
static const float MARKER_MM    = 15.f;
static const int   DICT_ID      = cv::aruco::DICT_5X5_100;
static const int   TARGET       = 20;
static const int   CAM_W        = 1920;
static const int   CAM_H        = 1080;
static const double DEG_PER_STEP = 360.0 / 4096.0;
static const int   SERVO_CENTER  = 2048;

static cv::Matx33d Rz(double deg) {
    double r = deg * 3.14159265358979323846 / 180.0;
    return { std::cos(r),-std::sin(r),0, std::sin(r),std::cos(r),0, 0,0,1 };
}
static cv::Matx33d Ry(double deg) {
    double r = deg * 3.14159265358979323846 / 180.0;
    return { std::cos(r),0,std::sin(r), 0,1,0, -std::sin(r),0,std::cos(r) };
}

static cv::Mat pose_to_matrix(const NexArmCoords& c) {
    double j1 = -(c.servo[0] - SERVO_CENTER) * DEG_PER_STEP;
    cv::Matx33d R = Rz(j1) * Ry(-c.pitch);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    for (int i=0;i<3;i++) for(int j=0;j<3;j++) T.at<double>(i,j)=R(i,j);
    T.at<double>(0,3)=c.x; T.at<double>(1,3)=c.y; T.at<double>(2,3)=c.z;
    return T;
}

struct Sample {
    cv::Mat R_g2b, t_g2b, R_t2c, t_t2c;
    double j1, pitch, x, y, z;
};

struct Coverage {
    int count=0; double pitch_range=0,j1_range=0,trans_range=0; int unique_poses=0;
    std::string hint;
};

static Coverage analyse(const std::vector<Sample>& S) {
    Coverage c; c.count=(int)S.size();
    if(S.empty()){c.hint="Move arm, board visible, then SPACE";return c;}
    double pmin=1e9,pmax=-1e9,j1min=1e9,j1max=-1e9,xmin=1e9,xmax=-1e9;
    std::set<std::pair<int,int>> uniq;
    for(auto& s:S){
        pmin=(std::min)(pmin,s.pitch);pmax=(std::max)(pmax,s.pitch);
        j1min=(std::min)(j1min,s.j1);j1max=(std::max)(j1max,s.j1);
        xmin=(std::min)(xmin,s.x);xmax=(std::max)(xmax,s.x);
        uniq.insert({(int)std::round(s.j1/5)*5,(int)std::round(s.pitch/5)*5});
    }
    c.pitch_range=pmax-pmin; c.j1_range=j1max-j1min;
    c.trans_range=xmax-xmin; c.unique_poses=(int)uniq.size();
    if(c.pitch_range<40)      c.hint="Tilt end-effector more (need 60deg pitch range)";
    else if(c.j1_range<60)    c.hint="Rotate base more left/right (need 90deg)";
    else if(c.trans_range<100)c.hint="Move arm farther across (need 150mm)";
    else if(c.unique_poses<12)c.hint="Good! Add more diverse poses.";
    else                      c.hint="Excellent! Press ESC to compute.";
    return c;
}

static void bar(cv::Mat& img,int x,int y,int w,float f,const std::string& lbl,bool ok){
    cv::rectangle(img,{x,y},{x+w,y+18},{50,50,50},-1);
    int fi=(std::min)((int)(f*w),w);
    if(fi>0) cv::rectangle(img,{x,y},{x+fi,y+18},ok?cv::Scalar(0,200,80):cv::Scalar(0,140,220),-1);
    cv::rectangle(img,{x,y},{x+w,y+18},{150,150,150},1);
    cv::putText(img,lbl,{x+w+8,y+14},cv::FONT_HERSHEY_SIMPLEX,0.48,{210,210,210},1);
}

static void save_json(const cv::Mat& T,double mean_mm,double max_mm){
    std::ofstream f("handeye.json"); f<<std::fixed;
    f<<"{\n  \"T_cam2gripper\": [\n";
    for(int i=0;i<4;i++){f<<"    [";for(int j=0;j<4;j++){if(j)f<<", ";f<<T.at<double>(i,j);}f<<(i<3?"],":"]")<<"\n";}
    f<<"  ],\n  \"mean_error_mm\": "<<mean_mm<<",\n  \"max_error_mm\": "<<max_mm<<"\n}\n";
}

int main(int argc, char* argv[]) {
    if(argc<2){std::cerr<<"Usage: calibrate_handeye.exe <COMx>\n";return 1;}

    auto cfg = NexArmConfig::auto_load();
    if(!cfg.cam_intrinsics_valid()){
        std::cerr<<"No intrinsics in config. Run calibrate_intrinsic first.\n";return 1;
    }
    cv::Mat K=(cv::Mat_<double>(3,3)<<cfg.cam_fx,0,cfg.cam_cx,0,cfg.cam_fy,cfg.cam_cy,0,0,1);
    cv::Mat dist=cv::Mat::zeros(5,1,CV_64F);

    auto dict  = cv::aruco::getPredefinedDictionary(DICT_ID);
    auto board = cv::aruco::CharucoBoard({BOARD_COLS,BOARD_ROWS},SQUARE_MM,MARKER_MM,dict);
    cv::aruco::CharucoDetector detector(board);

    OrbbecCamera cam(CAM_W,CAM_H,30);
    cam.open(); cam.warmup(5);

    NexArmClient arm(argv[1]);
    arm.open();

    std::vector<Sample> samples;
    std::cout<<"Move arm to different poses and press SPACE. ESC to compute.\n";

    while(true){
        auto frame=cam.capture();
        cv::Mat gray; cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        try{detector.detectBoard(gray,corners,ids);}catch(...){}
        bool found=!corners.empty()&&(int)corners.size()>=6;

        cv::Mat disp; cv::resize(frame,disp,{1280,720});
        float sx=1280.f/CAM_W,sy=720.f/CAM_H;
        if(found){
            std::vector<cv::Point2f> sc;
            for(auto& p:corners) sc.push_back({p.x*sx,p.y*sy});
            for(auto& p:sc) cv::circle(disp,p,4,{0,255,0},-1);
            cv::putText(disp,"corners="+std::to_string(corners.size()),
                {10,60},cv::FONT_HERSHEY_SIMPLEX,0.7,{0,220,0},2);
        }

        cv::rectangle(disp,{0,0},{760,160},{30,30,30},-1);
        Coverage cov=analyse(samples);
        bar(disp,10, 8,320,(float)cov.count/TARGET,
            "Samples: "+std::to_string(cov.count)+"/"+std::to_string(TARGET),cov.count>=TARGET);
        bar(disp,10,33,320,(float)(cov.pitch_range/60.0),
            "Pitch: "+std::to_string((int)cov.pitch_range)+"deg/60",cov.pitch_range>=60);
        bar(disp,10,58,320,(float)(cov.j1_range/90.0),
            "Base: "+std::to_string((int)cov.j1_range)+"deg/90",cov.j1_range>=90);
        bar(disp,10,83,320,(float)(cov.trans_range/150.0),
            "Trans: "+std::to_string((int)cov.trans_range)+"mm/150",cov.trans_range>=150);
        bar(disp,10,108,320,(float)cov.unique_poses/12.f,
            "Unique: "+std::to_string(cov.unique_poses)+"/12",cov.unique_poses>=12);
        cv::putText(disp,found?"DETECTED  SPACE=capture":"No board",
            {10,148},cv::FONT_HERSHEY_SIMPLEX,0.55,
            found?cv::Scalar(0,220,0):cv::Scalar(0,80,220),1);
        cv::putText(disp,cov.hint,{340,30},cv::FONT_HERSHEY_SIMPLEX,0.52,{200,200,0},1);

        cv::imshow("Hand-Eye Calibration",disp);
        int key=cv::waitKey(1)&0xFF;
        if(key==27) break;
        if(key==32&&found){
            arm.flush_rx();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            NexArmCoords c=arm.get_full_coords(2000);
            if(!c.has_full){std::cout<<"No full coords, skip\n";continue;}
            if(c.x < 50){std::cout<<"x="<<c.x<<"mm too small (arm facing backward), skip\n";continue;}

            // Estimate board pose via solvePnP on ChArUco corners
            std::vector<cv::Point3f> obj3d;
            const auto& all_corners = board.getChessboardCorners();
            for(int id:ids) obj3d.push_back(all_corners[id]);
            cv::Mat rvec,tvec;
            cv::solvePnP(obj3d,corners,K,dist,rvec,tvec);
            cv::Mat R; cv::Rodrigues(rvec,R);

            cv::Mat Tgb=pose_to_matrix(c);
            Sample s;
            // calibrateHandEye needs T_gripper2base = inv(T_base2gripper)
            cv::Mat Tg2b=Tgb.inv();
            s.R_g2b=Tg2b(cv::Rect(0,0,3,3)).clone();
            s.t_g2b=Tg2b(cv::Rect(3,0,1,3)).clone();
            R.convertTo(s.R_t2c,CV_64F);
            tvec.convertTo(s.t_t2c,CV_64F);
            s.j1=-(c.servo[0]-SERVO_CENTER)*DEG_PER_STEP;
            s.pitch=c.pitch;s.x=c.x;s.y=c.y;s.z=c.z;
            samples.push_back(s);
            std::cout<<"Captured "<<samples.size()
                     <<"  j1="<<(int)s.j1<<"deg  pitch="<<(int)s.pitch<<"deg\n";
        }
    }

    cam.close(); arm.close(); cv::destroyAllWindows();
    if((int)samples.size()<4){std::cerr<<"Not enough samples.\n";return 1;}

    // calibrateRobotWorldHandEye: solves AX=ZB, no need for accurate rotation model
    // R_w2c = board->cam (from solvePnP)
    // R_b2g = base->gripper = inv(pose_to_matrix)  [stored in R_g2b as gripper2base after inv()]
    std::vector<cv::Mat> R_w2c,t_w2c,R_b2g,t_b2g;
    for(auto& s:samples){
        R_w2c.push_back(s.R_t2c); t_w2c.push_back(s.t_t2c);
        // s.R_g2b/t_g2b is already T_gripper2base (we stored inv in Sample)
        R_b2g.push_back(s.R_g2b.t());  // base2gripper R = gripper2base R transposed
        cv::Mat tb2g = -(s.R_g2b.t()) * s.t_g2b;
        t_b2g.push_back(tb2g);
    }

    cv::Mat R_b2w,t_b2w,R_g2c,t_g2c;
    std::cout<<"calibrateRobotWorldHandEye (SHAH)...\n";
    cv::calibrateRobotWorldHandEye(R_w2c,t_w2c,R_b2g,t_b2g,
        R_b2w,t_b2w,R_g2c,t_g2c,
        cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH);

    // T_cam2gripper = inv(T_gripper2cam)
    cv::Mat Tg2c=cv::Mat::eye(4,4,CV_64F);
    R_g2c.copyTo(Tg2c(cv::Rect(0,0,3,3))); t_g2c.copyTo(Tg2c(cv::Rect(3,0,1,3)));
    cv::Mat best_T=Tg2c.inv();

    // Consistency evaluation
    std::vector<cv::Mat> pos;
    for(auto& s:samples){
        cv::Mat Tgb=cv::Mat::eye(4,4,CV_64F),Ttc=cv::Mat::eye(4,4,CV_64F);
        s.R_g2b.copyTo(Tgb(cv::Rect(0,0,3,3))); s.t_g2b.copyTo(Tgb(cv::Rect(3,0,1,3)));
        s.R_t2c.copyTo(Ttc(cv::Rect(0,0,3,3))); s.t_t2c.copyTo(Ttc(cv::Rect(3,0,1,3)));
        pos.push_back(cv::Mat((Tgb*best_T*Ttc)(cv::Rect(3,0,1,3))));
    }
    cv::Mat mean=cv::Mat::zeros(3,1,CV_64F);
    for(auto& p:pos)mean+=p; mean/=(double)pos.size();
    double sum2=0; for(auto& p:pos){cv::Mat d=p-mean;sum2+=d.dot(d);}
    double best_err=std::sqrt(sum2/pos.size())*1000.0;

    std::cout<<"\nConsistency: "<<best_err<<" mm\n"<<best_T<<"\n";
    save_json(best_T,best_err,best_err);
    std::cout<<"Saved: handeye.json\n";
    if(best_err>5.0) std::cout<<"WARNING: >5mm\n";
    return 0;
}
