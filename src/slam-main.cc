//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam_main.cc
\brief   Main entry point to process a ROS bag file through a vision SLAM
         frontend, and save the resulting SLAM dataset.
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>


#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"

using ros::Time;
using std::string;
using std::vector;
using Eigen::Quaternionf;
using Eigen::Vector3f;

void ProcessBagfile(const char* filename) {
  rosbag::Bag bag;
  try {
    bag.open(filename,rosbag::bagmode::Read);
  } catch(rosbag::BagException exception) {
    printf("Unable to read %s, reason:\n %s\n", filename, exception.what());
    return;
  }

  vector<string> topics;
  topics.push_back("localization");
  topics.push_back("laser");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  double bag_t_start = -1;
  // Iterate for every message.
  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    const rosbag::MessageInstance& message = *it;
    if (bag_t_start < 0.0) {
      bag_t_start = message.getTime().toSec();
    }
    {
      sensor_msgs::LaserScanPtr laser_msg =
          message.instantiate<sensor_msgs::LaserScan>();
      if (laser_msg != NULL) {
        printf("Laser t=%f\n", message.getTime().toSec());
      }
    }
    {
      laser_extractor::LocalizationMsgPtr localization_msg =
          message.instantiate<laser_extractor::LocalizationMsg>();
      if (localization_msg != NULL) {
        printf("Localization t=%f\n", message.getTime().toSec());
      }
    }
  }
}


int main(int argc, char** argv) {
  // OMP_PARALLEL_FOR
  for (int i = 1; i < argc; ++i) {
    printf("Converting %s\n", argv[i]);
    ProcessBagfile(argv[i]);
  }
  return 0;
}
