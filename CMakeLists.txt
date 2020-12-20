PROJECT(vision_slam_frontend)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

OPTION(USE_XFEATURES "Compile with OpenCV extra features module" OFF)

MESSAGE(STATUS "Compiling Slam Frontend")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

FIND_PACKAGE(OpenCV EXACT 3.2.0 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Ask ROS to compile custom message type definitions.
rosbuild_genmsg()

INCLUDE_DIRECTORIES(src/shared)
ADD_SUBDIRECTORY(src/shared)
LINK_DIRECTORIES(src/shared/build)

SET(libs roslib roscpp rosbag console_bridge cv_bridge glog gflags boost_system)

IF (USE_XFEATURES)
  MESSAGE(STATUS "Compiling with OpenCV xfeatures")
  ADD_DEFINITIONS(-DUSE_XFEATURES)
  SET(libs ${libs} opencv_xfeatures2d)
ENDIF ()

ROSBUILD_ADD_EXECUTABLE(slam_frontend
                        src/slam_frontend_main.cc
                        src/slam_frontend.cc
                        src/gui_helpers.cc)

TARGET_LINK_LIBRARIES(slam_frontend
                      ${AMRL_LIBRARY_NAME}
                      ${OpenCV_LIBS}
                      ${libs})

