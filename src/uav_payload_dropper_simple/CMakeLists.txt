cmake_minimum_required(VERSION 3.8)
project(uav_payload_dropper_simple)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(MAVSDK REQUIRED) 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(payload_dropper src/payload_dropper.cpp)

# target_include_directories(payload_dropper PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
#
# target_link_libraries(payload_dropper
#   MAVSDK::mavsdk
# )
#
# install(TARGETS payload_dropper
#   DESTINATION lib/${PROJECT_NAME}
# )
#

install(DIRECTORY include/
  DESTINATION include
) 

ament_package()
