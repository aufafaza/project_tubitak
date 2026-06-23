cmake_minimum_required(VERSION 3.8)
project(uav_sim)

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

ament_package()
