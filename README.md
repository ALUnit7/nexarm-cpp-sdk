# NexArm C++ SDK

C++ SDK for controlling the NexArm robotic arm via UART serial protocol.

**Platform**: Windows (Linux port path included via `#ifdef`)  
**Dependencies**: None (Win32 API only)  
**Standard**: C++17

## Files

```
nexarm.hpp            # Public API
nexarm.cpp            # Implementation (Windows serial backend)
nexarm_config.hpp     # Config loader (no dependencies)
nexarm_config.yaml    # Calibration config (edit to match your hardware)
test_nexarm.cpp       # Test program (--dry-run or hardware)
examples/
  ex01_status.cpp     # Read firmware, battery, current position
  ex02_move.cpp       # Absolute and incremental moves
  ex03_gripper.cpp    # Gripper open/close
  ex04_config.cpp     # Auto-load config, gripper polarity
```

## Build

```bash
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64   # or use CLion directly
cmake --build . --config Release
```

## Quick Start

```cpp
#include "nexarm.hpp"
#include "nexarm_config.hpp"
#include <thread>
#include <chrono>
using namespace std::chrono_literals;

int main() {
    auto cfg = NexArmConfig::auto_load();   // loads nexarm_config.yaml automatically
    NexArmClient arm("COM5");
    arm.open();

    arm.set_pose(200, 0, 200, 0, 0, 0, 1500);          // move to home
    std::this_thread::sleep_for(1800ms);

    arm.set_pose(200, 0, 200, -45, 0,
                 (float)cfg.gripper_open_value(), 500); // open gripper
    std::this_thread::sleep_for(700ms);

    arm.close();
}
```

## Config

`nexarm_config.yaml` is auto-loaded from the executable directory. Edit to match your hardware:

```yaml
gripper:
  polarity: 1       # -1 if open/close is reversed
  open_value: -80   # claw value for fully open
  close_value: 0    # claw value for fully closed
```

## Test

```bash
# No hardware required
test_nexarm.exe --dry-run

# With hardware (close upper-computer software first)
test_nexarm.exe COM5
ex01_status.exe COM5
ex02_move.exe   COM5
ex03_gripper.exe COM5
ex04_config.exe COM5
```

## API Reference

| Method | Description |
|--------|-------------|
| `set_pose(x,y,z,pitch,roll,claw,ms)` | Absolute move (mm, deg) |
| `move_increment(dx,dy,dz,dp,dr,dc,ms)` | Incremental move |
| `get_full_coords()` | Read x/y/z/pitch/roll/claw + 6 servo positions |
| `get_current_coords()` | Read x/y/z (query-reply) |
| `get_firmware_version()` | e.g. `"1.0.0"` |
| `get_battery_voltage()` | mV |
| `set_buzzer(on,off,times,freq)` | Buzzer control |
| `flush_rx()` | Clear receive buffer |

## Linux Port

Replace the 4 serial functions in `nexarm.cpp` (`open`, `serial_read`, `serial_write`, `flush_rx`) with termios equivalents. Change port from `"COM5"` to `"/dev/ttyUSB0"`. Everything else is platform-independent.

## Protocol

Frame format: `FF FF [ID] [LEN] [CMD] [PAYLOAD...] [CHECKSUM]`  
Checksum: `~(ID + LEN + CMD + sum(PAYLOAD)) & 0xFF`  
Serial: 1 000 000 baud, 8N1

See `README.md` protocol section or `nexarm.hpp` command codes for full details.
