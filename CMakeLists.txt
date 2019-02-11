PROJECT(vision_slam_frontend)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Slam Frontend")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

FIND_PACKAGE(OpenCV REQUIRED)
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

SET(libs roslib roscpp rosbag glog gflags opencv_xfeatures2d)

ADD_LIBRARY(shared_library
            src/gui_helpers.cc
            src/timer.cc)

ROSBUILD_ADD_EXECUTABLE(slam_frontend
                        src/slam_frontend_main.cc
                        src/slam_frontend.cc
                        src/slam_frame.cc
                        src/slam_config.cc
                        src/slam_frontend_backend_comm.cc)

TARGET_LINK_LIBRARIES(slam_frontend
                      shared_library
                      ${OpenCV_LIBS}
                      ${libs})
                      
ROSBUILD_ADD_EXECUTABLE(slam_test
			src/test/slam_test.cc)

            
TARGET_LINK_LIBRARIES(slam_test
		      ${OpenCV_LIBS}
		      ${libs})
		      
ROSBUILD_ADD_EXECUTABLE(bag_image_extract
			src/test/bag_extract.cc)
			
TARGET_LINK_LIBRARIES(bag_image_extract
		      ${OpenCV_LIBS}
		      ${libs})

