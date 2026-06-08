#include "orbbec_camera.hpp"
#include <vector>

OrbbecCamera::OrbbecCamera(int width, int height, int fps)
    : m_width(width), m_height(height), m_fps(fps) {}

OrbbecCamera::~OrbbecCamera() { close(); }

void OrbbecCamera::open() {
    if (m_pipeline) return;
    m_pipeline = std::make_shared<ob::Pipeline>();

    auto profiles = m_pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    try {
        m_profile = profiles->getVideoStreamProfile(m_width, m_height, OB_FORMAT_RGB, m_fps);
    } catch (...) {
        m_profile = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
    }

    try {
        auto intr = m_profile->getIntrinsic();
        m_intr = { intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height };
    } catch (...) {
        // Gemini first-gen may not expose intrinsics via SDK; use defaults
        m_intr = { 0, 0, 0, 0, m_width, m_height };
    }

    auto cfg = std::make_shared<ob::Config>();
    cfg->enableStream(m_profile);
    m_pipeline->start(cfg);
}

void OrbbecCamera::close() {
    if (!m_pipeline) return;
    m_pipeline->stop();
    m_pipeline.reset();
    m_profile.reset();
}

cv::Mat OrbbecCamera::capture(int timeout_ms) {
    for (int i = 0; i < 30; ++i) {
        auto fs = m_pipeline->waitForFrames(timeout_ms);
        if (!fs) continue;
        auto cf = fs->colorFrame();
        if (!cf) continue;

        int w = cf->width(), h = cf->height();
        auto fmt = cf->format();
        const uint8_t* data = (const uint8_t*)cf->data();
        uint32_t sz = cf->dataSize();

        if (fmt == OB_FORMAT_RGB) {
            cv::Mat rgb(h, w, CV_8UC3, (void*)data);
            cv::Mat bgr;
            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
            return bgr.clone();
        } else if (fmt == OB_FORMAT_MJPG) {
            std::vector<uint8_t> buf(data, data + sz);
            return cv::imdecode(buf, cv::IMREAD_COLOR);
        }
    }
    throw std::runtime_error("OrbbecCamera: failed to get color frame");
}

void OrbbecCamera::warmup(int frames) {
    for (int i = 0; i < frames; ++i) {
        auto fs = m_pipeline->waitForFrames(500);
        (void)fs;
    }
}

CameraIntrinsics OrbbecCamera::intrinsics() const { return m_intr; }
