// test_depth.cpp - color + depth streams, click to get 3D coordinates
// Usage: test_depth.exe
#include "../nexarm_config.hpp"
#include <libobsensor/ObSensor.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

struct State { cv::Mat depth_mm; float fx,fy,cx,cy; };
static State g;

static void onMouse(int ev, int x, int y, int, void*) {
    if (ev!=cv::EVENT_LBUTTONDOWN||g.depth_mm.empty()||g.fx==0) return;
    float d=g.depth_mm.at<uint16_t>(y,x);
    if (d<100||d>5000){std::cout<<"No valid depth at ("<<x<<","<<y<<")\n";return;}
    float Z=d, X=(x-g.cx)*Z/g.fx, Y=(y-g.cy)*Z/g.fy;
    std::cout<<"Pixel("<<x<<","<<y<<")  depth="<<(int)d<<"mm  3D=["
             <<(int)X<<","<<(int)Y<<","<<(int)Z<<"] mm\n";
}

int main() {
    auto cfg = NexArmConfig::auto_load();

    ob::Pipeline pipe;
    auto device = pipe.getDevice();

    // Enable disparity→depth conversion for Gemini first-gen
    try {
        if (device->isPropertySupported(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL,
                OB_PERMISSION_READ_WRITE))
            device->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, true);
    } catch (...) {}

    auto config = std::make_shared<ob::Config>();

    // Color stream
    {
        auto profs = pipe.getStreamProfileList(OB_SENSOR_COLOR);
        auto p = profs->getVideoStreamProfile(1280,720,OB_FORMAT_RGB,30);
        if (!p) p = profs->getProfile(0)->as<ob::VideoStreamProfile>();
        config->enableStream(p);
    }

    // Depth stream
    {
        auto profs = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
        std::shared_ptr<ob::VideoStreamProfile> p;
        for (int i=0; i<(int)profs->count(); ++i) {
            auto pp = profs->getProfile(i)->as<ob::VideoStreamProfile>();
            std::cout<<"  depth profile "<<i<<": "<<pp->width()<<"x"<<pp->height()
                     <<" fmt="<<pp->format()<<" fps="<<pp->fps()<<"\n";
            if (!p && pp->format()==OB_FORMAT_Y16) p=pp;
        }
        if (!p) p = profs->getProfile(0)->as<ob::VideoStreamProfile>();
        config->enableStream(p);
        std::cout<<"Using depth: "<<p->width()<<"x"<<p->height()<<"\n";
    }

    pipe.start(config);

    // Get intrinsics
    try {
        auto param = pipe.getCameraParam();
        auto& di = param.depthIntrinsic;
        if (di.fx > 0) {
            g.fx=di.fx; g.fy=di.fy; g.cx=di.cx; g.cy=di.cy;
            std::cout<<"Depth intrinsics from SDK: fx="<<g.fx<<" fy="<<g.fy<<"\n";
        }
    } catch (...) {}

    // Fallback: use color intrinsics scaled to depth resolution
    if (g.fx==0 && cfg.cam_intrinsics_valid()) {
        // Assume depth is 640x400 or similar; scale from 1920x1080
        g.fx=cfg.cam_fx*640.f/cfg.cam_width; g.fy=cfg.cam_fy*400.f/cfg.cam_height;
        g.cx=cfg.cam_cx*640.f/cfg.cam_width; g.cy=cfg.cam_cy*400.f/cfg.cam_height;
        std::cout<<"Using color intrinsics scaled: fx="<<g.fx<<"\n";
    }

    cv::namedWindow("Depth Test");
    cv::setMouseCallback("Depth Test", onMouse);
    std::cout<<"Click image to get 3D coord. ESC=quit\n";

    cv::Mat color_disp;
    while (true) {
        auto fs = pipe.waitForFrames(200);
        if (!fs) continue;

        // Color
        auto cf = fs->colorFrame();
        if (cf) {
            cv::Mat rgb(cf->height(),cf->width(),CV_8UC3,cf->data());
            cv::cvtColor(rgb, color_disp, cv::COLOR_RGB2BGR);
        }

        // Depth
        auto df = fs->depthFrame();
        cv::Mat disp = color_disp.empty() ? cv::Mat::zeros(720,1280,CV_8UC3) : color_disp.clone();
        if (df) {
            float scale=df->getValueScale();
            cv::Mat d16(df->height(),df->width(),CV_16UC1,df->data());
            cv::Mat dsc; d16.convertTo(dsc,CV_16UC1,scale);
            // Resize depth to match display
            cv::resize(dsc, g.depth_mm, {1280,720});
            // Update intrinsics pixel coords for display resolution
            if (g.fx>0 && df->width()!=1280) {
                float sx=1280.f/df->width(), sy=720.f/df->height();
                // Only need to update cx/cy for display coords; fx/fy stay for actual 3D
            }
            cv::Mat dvis; cv::normalize(g.depth_mm,dvis,0,255,cv::NORM_MINMAX,CV_8U);
            cv::applyColorMap(dvis,dvis,cv::COLORMAP_JET);
            if (!disp.empty()) cv::addWeighted(disp,0.6,dvis,0.4,0,disp);
            else disp=dvis;
            cv::putText(disp,"DEPTH OK - Click=3D",{10,30},cv::FONT_HERSHEY_SIMPLEX,0.7,{0,220,0},2);
        } else {
            cv::putText(disp,"No depth",{10,30},cv::FONT_HERSHEY_SIMPLEX,0.7,{0,80,220},2);
        }
        cv::imshow("Depth Test",disp);
        if ((cv::waitKey(1)&0xFF)==27) break;
    }
    pipe.stop();
    return 0;
}
