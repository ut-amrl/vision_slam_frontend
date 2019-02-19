#!/bin/bash
source /opt/ros/kinetic/setup.bash
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
mkdir build
make -j8
