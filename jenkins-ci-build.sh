#!/bin/bash
source /opt/ros/kinetic/setup.bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
rm -rf build lib bin msg_gen
mkdir build
make -j8
