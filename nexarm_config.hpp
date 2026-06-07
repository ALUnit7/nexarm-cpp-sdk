#pragma once
// Minimal NexArm config loader — no external dependencies
// Parses a subset of YAML: sections, key: value pairs, lists on one line

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <limits.h>
#endif

struct NexArmConfig {
    // Gripper
    int   gripper_polarity  = 1;
    int   gripper_open      = -80;
    int   gripper_close     = 0;

    // TCP offset (mm)
    float tcp_x = 0.f, tcp_y = 0.f, tcp_z = 0.f;

    // Joint limits
    int   x_min=-300, x_max=300;
    int   y_min=-300, y_max=300;
    int   z_min=50,   z_max=400;
    float pitch_min=-180.f, pitch_max=90.f;

    // Hand-eye (T_cam2gripper row-major 4x4)
    bool  hand_eye_enabled = false;
    float hand_eye[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

    // Helper: apply gripper polarity
    int gripper_open_value()  const { return gripper_polarity > 0 ? gripper_open  : gripper_close; }
    int gripper_close_value() const { return gripper_polarity > 0 ? gripper_close : gripper_open;  }

    // Auto-load: search exe dir, then working dir
    static NexArmConfig auto_load(const std::string& filename = "nexarm_config.yaml") {
        std::vector<std::string> candidates;

        // 1. Same directory as the executable
        candidates.push_back(exe_dir() + "/" + filename);
        // 2. Working directory
        candidates.push_back(filename);

        for (const auto& p : candidates) {
            std::ifstream test(p);
            if (test.good()) {
                std::cout << "[config] Loaded: " << p << "\n";
                return load(p);
            }
        }
        std::cout << "[config] Not found, using defaults\n";
        return NexArmConfig{};
    }

    static NexArmConfig load(const std::string& path) {
        NexArmConfig cfg;
        std::ifstream f(path);
        if (!f) throw std::runtime_error("Cannot open config: " + path);

        std::string section, line;
        while (std::getline(f, line)) {
            // strip comment and whitespace
            auto ci = line.find('#');
            if (ci != std::string::npos) line = line.substr(0, ci);
            auto trimmed = trim(line);
            if (trimmed.empty()) continue;

            // section header (no colon, ends with colon)
            if (trimmed.back() == ':' && trimmed.find(':') == trimmed.size()-1) {
                section = trimmed.substr(0, trimmed.size()-1);
                continue;
            }

            auto colon = trimmed.find(':');
            if (colon == std::string::npos) continue;
            std::string key = trim(trimmed.substr(0, colon));
            std::string val = trim(trimmed.substr(colon + 1));

            if (section == "gripper") {
                if (key=="polarity")    cfg.gripper_polarity = stoi(val);
                if (key=="open_value")  cfg.gripper_open     = stoi(val);
                if (key=="close_value") cfg.gripper_close    = stoi(val);
            } else if (section == "tcp_offset") {
                if (key=="x") cfg.tcp_x = stof(val);
                if (key=="y") cfg.tcp_y = stof(val);
                if (key=="z") cfg.tcp_z = stof(val);
            } else if (section == "joint_limits") {
                if (key=="x_min")     cfg.x_min     = stoi(val);
                if (key=="x_max")     cfg.x_max     = stoi(val);
                if (key=="y_min")     cfg.y_min     = stoi(val);
                if (key=="y_max")     cfg.y_max     = stoi(val);
                if (key=="z_min")     cfg.z_min     = stoi(val);
                if (key=="z_max")     cfg.z_max     = stoi(val);
                if (key=="pitch_min") cfg.pitch_min = stof(val);
                if (key=="pitch_max") cfg.pitch_max = stof(val);
            } else if (section == "hand_eye") {
                if (key=="enabled") cfg.hand_eye_enabled = (val=="true");
                if (key=="matrix") {
                    // parse inline list: [v0, v1, ...]
                    auto s = val; s.erase(std::remove(s.begin(),s.end(),'['),s.end());
                               s.erase(std::remove(s.begin(),s.end(),']'),s.end());
                    std::istringstream ss(s);
                    std::string tok; int i=0;
                    while (std::getline(ss,tok,',') && i<16)
                        cfg.hand_eye[i++] = std::stof(trim(tok));
                }
            }
        }
        return cfg;
    }

private:
    static std::string trim(const std::string& s) {
        size_t a = s.find_first_not_of(" \t\r\n");
        size_t b = s.find_last_not_of(" \t\r\n");
        return (a==std::string::npos) ? "" : s.substr(a, b-a+1);
    }

    static std::string exe_dir() {
#ifdef _WIN32
        char buf[MAX_PATH];
        DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
        if (!n) return ".";
        std::string p(buf, n);
        auto pos = p.find_last_of("\\/");
        return pos == std::string::npos ? "." : p.substr(0, pos);
#else
        char buf[PATH_MAX];
        ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf)-1);
        if (n <= 0) return ".";
        buf[n] = '\0';
        std::string p(buf);
        auto pos = p.find_last_of('/');
        return pos == std::string::npos ? "." : p.substr(0, pos);
#endif
    }
};
