cmake_minimum_required(VERSION 3.8)
project(cam_calib)

find_package(OpenCV REQUIRED)

add_executable(cam_calib cam_calib.cpp)

target_include_directories(cam_calib PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cam_calib ${OpenCV_LIBS})
