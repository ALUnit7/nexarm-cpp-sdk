# OrbbecSDK Linux 安装说明

## 方法1：使用 .deb 包安装（推荐）

本目录包含：
- `OrbbecSDK_v1.10.18_amd64.deb` — Linux x64 安装包
- `OrbbecSDK_C_C++_v1.10.27_..._linux_x64_release.zip` — C/C++ 开发包

### x64 Ubuntu 安装：
```bash
sudo dpkg -i OrbbecSDK_v1.10.18_amd64.deb
# 头文件和库安装到 /usr/local
dpkg -L orbbecsdk
```

### Arm64 安装：
从 https://github.com/orbbec/OrbbecSDK/releases 下载：
`OrbbecSDK_v1.x.x_arm64.deb`
```bash
sudo dpkg -i OrbbecSDK_v1.x.x_arm64.deb
```

## 方法2：从 C/C++ zip 包使用

```bash
unzip OrbbecSDK_C_C++_v1.10.27_*_linux_x64_release.zip
# 解压后包含 SDK/ 目录（include + lib）
```

在 `nexarm_cpp/CMakeLists.txt` 里设置路径：
```cmake
set(ORBBEC_SDK_DIR "/path/to/OrbbecSDK_linux/SDK")
```

## udev 规则（必须安装，否则无权限访问相机）

```bash
cd OrbbecSDK/misc/scripts
sudo chmod +x install_udev_rules.sh
sudo ./install_udev_rules.sh
sudo udevadm control --reload && sudo udevadm trigger
```

## 编译 nexarm_cpp 项目

```bash
cd nexarm_cpp
mkdir build && cd build
cmake .. -DORBBEC_SDK_DIR=/usr/local \
         -DOpenCV_DIR=/usr/lib/cmake/opencv4
make -j4
```

## 串口权限

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
# 然后把代码里的 "COM11" 改为 "/dev/ttyUSB0"
```
