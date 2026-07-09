cmake_minimum_required(VERSION 3.8)
project(uav_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(uav_vision SHARED src/segmentation.cpp src/georeference.cpp)

target_include_directories(uav_vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(uav_vision ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(detect_test src/test/detect_test.cpp)
ament_target_dependencies(detect_test rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge)
target_link_libraries(detect_test uav_vision)

install(TARGETS uav_vision
  EXPORT uav_vision_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS detect_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(uav_vision_export HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV Eigen3)

ament_package()
