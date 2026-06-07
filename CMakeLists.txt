cmake_minimum_required(VERSION 3.14)
project(nexarm_cpp CXX)
set(CMAKE_CXX_STANDARD 17)

add_library(nexarm STATIC nexarm.cpp)
target_include_directories(nexarm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_nexarm test_nexarm.cpp)
target_link_libraries(test_nexarm PRIVATE nexarm)

foreach(ex 01_status 02_move 03_gripper 04_config)
    add_executable(ex${ex} examples/ex${ex}.cpp)
    target_link_libraries(ex${ex} PRIVATE nexarm)
endforeach()

# Copy config file to build output directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/nexarm_config.yaml
               ${CMAKE_CURRENT_BINARY_DIR}/nexarm_config.yaml COPYONLY)

# Windows: no extra libs needed (uses Win32 API)
# Linux: uncomment if needed
# target_compile_definitions(nexarm PRIVATE _POSIX_C_SOURCE=200809L)
