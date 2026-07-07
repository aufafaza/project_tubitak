cmake_minimum_required(VERSION 3.8)
project(uav_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(airdrop_planning SHARED
  src/airdrop_planning/airdrop_manager.cpp
  src/airdrop_planning/airdrop_plan_fsm.cpp
  src/airdrop_planning/ballistic_model.cpp
  src/airdrop_planning/dubins_planner.cpp
)

target_include_directories(airdrop_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/airdrop_planning>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(airdrop_planning
  MAVSDK::mavsdk
  Eigen3::Eigen
)

install(TARGETS airdrop_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
