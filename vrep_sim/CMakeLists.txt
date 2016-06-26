project(motion-planning)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -g")
set(VREP_DIR "/home/lakshayg/bin/V-REP/")
set(VREP_INCLUDE "${VREP_DIR}/programming/include")
set(VREP_REMOTE_API "${VREP_DIR}/programming/remoteApi")
include_directories(${VREP_REMOTE_API} ${VREP_INCLUDE})

add_library(vrep_lib src/vrep.cpp)
target_link_libraries(vrep_lib pthread)

add_executable(vrep_client src/vrep_client.cpp)
target_link_libraries(vrep_client vrep_lib)
