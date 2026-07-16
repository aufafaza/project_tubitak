cmake_minimum_required(VERSION 3.8)
project(uav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(uav_vision REQUIRED)

add_library(uav_control SHARED src/uav_control.cpp)

target_include_directories(uav_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(uav_control
  MAVSDK::mavsdk
  Eigen3::Eigen
)

ament_target_dependencies(uav_control rclcpp)

install(TARGETS uav_control
  EXPORT uav_control_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(uav_control_export HAS_LIBRARY_TARGET)
ament_export_dependencies(MAVSDK Eigen3 rclcpp)

add_executable(mission_test src/mission_test.cpp)
target_link_libraries(mission_test uav_control ${OpenCV_LIBS})
ament_target_dependencies(mission_test rclcpp sensor_msgs cv_bridge uav_vision mavros_msgs)

install(TARGETS mission_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
