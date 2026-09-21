cmake_minimum_required(VERSION 3.10)
project(airsim_landing_camera_bridge LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
)
find_package(yaml-cpp REQUIRED)

set(AIRSIM_PLUGIN_ROOT "" CACHE PATH "Path to the landing-local AirSim plugin")
if(NOT EXISTS "${AIRSIM_PLUGIN_ROOT}/Source/AirLib/include/api/RpcLibClientBase.hpp")
  message(FATAL_ERROR "Invalid AIRSIM_PLUGIN_ROOT: ${AIRSIM_PLUGIN_ROOT}")
endif()

set(AIRLIB "${AIRSIM_PLUGIN_ROOT}/Source/AirLib")
add_executable(airsim_landing_camera_bridge airsim_landing_camera_bridge.cpp)
target_compile_features(airsim_landing_camera_bridge PRIVATE cxx_std_17)
target_compile_options(airsim_landing_camera_bridge PRIVATE -O3 -Wall -Wextra -Wpedantic)
target_include_directories(airsim_landing_camera_bridge PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(airsim_landing_camera_bridge SYSTEM PRIVATE
  "${AIRLIB}/include"
  "${AIRLIB}/deps/eigen3"
  "${AIRLIB}/deps/rpclib/include"
)
target_link_libraries(airsim_landing_camera_bridge PRIVATE
  ${catkin_LIBRARIES}
  yaml-cpp
  "${AIRSIM_PLUGIN_ROOT}/LandingGccLibs/libAirLib.a"
  "${AIRSIM_PLUGIN_ROOT}/LandingGccLibs/librpc.a"
  pthread
  stdc++fs
)
