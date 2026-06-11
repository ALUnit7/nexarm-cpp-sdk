# NexArm C++ SDK

C++ SDK for NexArm robotic arm (UART) + Orbbec Gemini camera integration.

**Platform**: Windows (MSVC) / Linux (GCC) via `#ifdef`  
**Standard**: C++17  
**Dependencies**: OpenCV 4.x, OrbbecSDK v1

## Project Structure

```
nexarm.hpp/cpp              # NexArm arm serial SDK
nexarm_config.hpp           # Config loader (YAML, no deps)
nexarm_config.yaml          # Calibration: intrinsics, hand-eye, gripper
camera/orbbec_camera.hpp/cpp  # OrbbecSDK v1 C++ wrapper
examples/                   # Usage examples
calibration/                # Calibration tools
test_nexarm.cpp             # Hardware test (--dry-run or COMx)
```

## Build — Windows (MSVC)

```bash
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64  # or use CLion
cmake --build . --config Release
```

Required paths (set via CMake cache or edit CMakeLists.txt):
- `OpenCV_DIR` = `C:/opencv/opencv/build`
- `ORBBEC_SDK_DIR` = path to OrbbecSDK v1.10.x

## Build — Linux (GCC)

```bash
# Install dependencies
sudo apt install libopencv-dev cmake build-essential

# Clone/copy OrbbecSDK v1 for Linux (from orbbec/OrbbecSDK releases)
# Set ORBBEC_SDK_DIR to the extracted path

mkdir build && cd build
cmake .. -DORBBEC_SDK_DIR=/path/to/OrbbecSDK \
         -DOpenCV_DIR=/usr/lib/cmake/opencv4
make -j4
```

**Linux serial port**: Change `"COM11"` → `"/dev/ttyUSB0"` in your code.  
The serial backend in `nexarm.cpp` has `#ifdef _WIN32` / `#else` blocks — no code changes needed, just the port string.

**Linux udev rules** (for Orbbec camera permissions):
```bash
cd OrbbecSDK/misc/scripts
sudo ./install_udev_rules.sh
sudo udevadm control --reload && sudo udevadm trigger
```

## Quick Start

```cpp
#include "nexarm.hpp"
#include "nexarm_config.hpp"

auto cfg = NexArmConfig::auto_load();   // loads nexarm_config.yaml
NexArmClient arm("COM11");             // Linux: "/dev/ttyUSB0"
arm.open();
arm.set_pose(200, 0, 200, 0, 0, 0, 1500);
std::this_thread::sleep_for(std::chrono::milliseconds(1800));
arm.close();
```

## Examples

| Binary | Description |
|--------|-------------|
| `test_nexarm --dry-run` | Frame encoding test, no hardware |
| `test_nexarm COM11` | Full hardware test |
| `ex01_status` | Read firmware, battery, position |
| `ex02_move` | Absolute + incremental moves |
| `ex03_gripper` | Gripper open/close |
| `ex04_config` | Config-based gripper control |
| `ex05_camera` | Live camera view (Orbbec) |
| `ex06_right_down` | Move arm to right side, gripper down |

## Calibration Tools

| Binary | Description |
|--------|-------------|
| `calibrate_intrinsic` | Camera intrinsic calibration (ChArUco, saves intrinsics.json) |
| `calibrate_offset` | Measure camera-gripper offset (S=visual, R=actual, ESC=result) |
| `measure_compensate` | Verify T_cam2gripper with ChArUco board |
| `detect_move_red` | Detect red square + move arm above it |
| `test_depth` | Depth stream test, click for 3D coords |

## Config File (`nexarm_config.yaml`)

Auto-loaded from executable directory. Key fields:

```yaml
gripper:
  polarity: 1        # -1 if open/close reversed
  open_value: -80
  close_value: 0

camera_intrinsics:
  fx: 1350.96  fy: 1351.34  cx: 980.6  cy: 564.5
  width: 1920  height: 1080
  dist: [0.098, -0.234, 0.002, 0.001, 0.132]

hand_eye:
  enabled: true
  # T_cam2gripper 4x4 row-major (manually measured or from calibrate_offset)
  matrix: [1,0,0,115.26, 0,1,0,0, 0,0,1,106.12, 0,0,0,1]
```

## Protocol Reference

Frame: `FF FF [ID] [LEN] [CMD] [PAYLOAD...] [CHECKSUM]`  
Checksum: `~(ID + LEN + CMD + sum(PAYLOAD)) & 0xFF`  
Serial: 1,000,000 baud, 8N1

## Linux Port Notes

Only `nexarm.cpp` needs changes for Linux (serial backend). The 4 functions:
- `open()` — use `termios` instead of `CreateFile`
- `serial_read()` — use `select()` + `read()`
- `serial_write()` — use `write()`
- `flush_rx()` — use `tcflush(TCIFLUSH)`

All marked with `#ifdef _WIN32` / `#else` — stubs already present.
