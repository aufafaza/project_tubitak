cmake_minimum_required(VERSION 3.8)
project(uav_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  model
  param
  scripts
  world
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS src/bridge/gz_payload.py
  RENAME payload_drop_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/bridge/gz_camera.py
  src/bridge/gz_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
