cmake_minimum_required(VERSION 3.14)
project(nexarm_cpp CXX)
set(CMAKE_CXX_STANDARD 17)

# ── Configurable paths (override via -D on command line or CLion CMake options) ──
set(OpenCV_DIR    "C:/opencv/opencv/build"  CACHE PATH "OpenCV build dir")
set(ORBBEC_SDK_DIR
    "G:/Robot/RCOM/OrbbecSDK_C_C++_v1.10.27_20250925_0549823cb_win_x64_release/OrbbecSDK_C_C++_v1.10.27_20250925_0549823cb_win_x64_release/OrbbecSDK_v1.10.27"
    CACHE PATH "OrbbecSDK root")

# ── OpenCV ────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)

# ── OrbbecSDK ─────────────────────────────────────────────────────────────────
set(ORBBEC_INCLUDE "${ORBBEC_SDK_DIR}/SDK/include")
set(ORBBEC_LIB     "${ORBBEC_SDK_DIR}/SDK/lib/OrbbecSDK.lib")

# ── NexArm SDK library ────────────────────────────────────────────────────────
add_library(nexarm STATIC nexarm.cpp)
target_include_directories(nexarm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ── Camera library ────────────────────────────────────────────────────────────
add_library(orbbec_cam STATIC camera/orbbec_camera.cpp)
target_include_directories(orbbec_cam PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/camera
    ${ORBBEC_INCLUDE}
    ${OpenCV_INCLUDE_DIRS})
target_link_libraries(orbbec_cam PUBLIC ${ORBBEC_LIB} ${OpenCV_LIBS})

# ── Helper macro: copy DLLs next to a target after build ─────────────────────
function(copy_dlls_to_target target glob_pattern)
    file(GLOB _dlls "${glob_pattern}")
    foreach(_dll ${_dlls})
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_dll}" "$<TARGET_FILE_DIR:${target}>/"
            COMMENT "Copying ${_dll}")
    endforeach()
endfunction()

# ── Test & examples ───────────────────────────────────────────────────────────
add_executable(test_nexarm test_nexarm.cpp)
target_link_libraries(test_nexarm PRIVATE nexarm)

foreach(ex 01_status 02_move 03_gripper 04_config)
    add_executable(ex${ex} examples/ex${ex}.cpp)
    target_link_libraries(ex${ex} PRIVATE nexarm)
endforeach()

add_executable(ex05_camera examples/ex05_camera.cpp)
target_link_libraries(ex05_camera PRIVATE nexarm orbbec_cam)

# ── Calibration tools ─────────────────────────────────────────────────────────
add_executable(calibrate_intrinsic calibration/calibrate_intrinsic.cpp)
target_link_libraries(calibrate_intrinsic PRIVATE orbbec_cam)

add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
target_link_libraries(calibrate_handeye PRIVATE nexarm orbbec_cam)

add_executable(detect_dict calibration/detect_dict.cpp)
target_link_libraries(detect_dict PRIVATE orbbec_cam)

add_executable(debug_ab calibration/debug_ab.cpp)
target_link_libraries(debug_ab PRIVATE nexarm orbbec_cam)

add_executable(debug_pitch_only calibration/debug_pitch_only.cpp)
target_link_libraries(debug_pitch_only PRIVATE nexarm orbbec_cam)

# Copy DLLs for camera targets (post-build)
foreach(cam_target ex05_camera calibrate_intrinsic calibrate_handeye detect_dict debug_ab debug_pitch_only)
    copy_dlls_to_target(${cam_target} "${ORBBEC_SDK_DIR}/SDK/lib/*.dll")
    copy_dlls_to_target(${cam_target} "${OpenCV_DIR}/x64/vc16/bin/opencv_world4120*.dll")
endforeach()

# ── Config file ───────────────────────────────────────────────────────────────
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/nexarm_config.yaml
               ${CMAKE_CURRENT_BINARY_DIR}/nexarm_config.yaml COPYONLY)
