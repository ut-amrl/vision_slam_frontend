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

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CompressedImage.h"
#include "nav_msgs/Odometry.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"

#include "slam_frontend.h"

using ros::Time;
using std::string;
using std::vector;
using Eigen::Quaternionf;
using Eigen::Vector3f;

DEFINE_string(image_topic, "camera", "Name of ROS topic for image data");
DEFINE_string(odom_topic, "odometry", "Name of ROS topic for odometry data");
DEFINE_string(input, "", "Name of ROS bag file to load");
DEFINE_bool(visualize, false, "Display images loaded");
DECLARE_string(helpon);
DECLARE_int32(v);

/* TODO List:
 * 1) Work with several images, allowing keypoints to come back (Done)
 * Tonight:
  * 2) Some way to store the image chains in a database.
  * 	2a) Each chain / track should have the same starting node and thats all we care about.
  * 3) Somehow broadcast each match to a ros node. (DONE)
  * 4) Somehow finally broadcast the database over a message. (DONE)
 * Tomorrow:
  * 5) Build a test application that given a bag file containing the database, verify matching.
  * 6) Use ConfigReader to be able to have lua file configs.
 */

void CompressedImageCallback(slam::Frontend& slam_frontend,  sensor_msgs::CompressedImage& msg) {
  double image_time = msg.header.stamp.toSec();
  if (FLAGS_v > 0) {
    printf("CompressedImage t=%f\n", image_time);
  }
  cv::Mat image = cv::imdecode(cv::InputArray(msg.data),1);
  if (msg.format.find("bayer_rggb8") != string::npos) {
    cv::Mat1b *image_channels = new cv::Mat1b[image.channels()];
    cv::split(image, image_channels);
    image = image_channels[0];
    cv::cvtColor(image_channels[0], image, cv::COLOR_BayerBG2BGR);
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    delete [] image_channels;
  }
  slam_frontend.ObserveImage(image, image_time);
  if (FLAGS_visualize) {
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    if (cv::waitKey(16) == 27) {
      exit(0);
    }
  }
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
}

void ProcessBagfile(const char* filename, ros::NodeHandle* n) {
  rosbag::Bag bag;
  try {
    bag.open(filename,rosbag::bagmode::Read);
  } catch(rosbag::BagException exception) {
    printf("Unable to read %s, reason:\n %s\n", filename, exception.what());
    return;
  }
  printf("Processing %s\n", filename);
  // ros::Subscriber image_sub = n->subscribe("image_convert_callback", 1,)

  vector<string> topics;
  topics.push_back(FLAGS_image_topic.c_str());
  topics.push_back(FLAGS_odom_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  slam::Frontend slam_frontend(*n, "");

  double bag_t_start = -1;
  // Iterate for every message.
  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    const rosbag::MessageInstance& message = *it;
    if (bag_t_start < 0.0) {
      bag_t_start = message.getTime().toSec();
    }
    ros::spinOnce();
    {
      sensor_msgs::CompressedImagePtr image_msg =
          message.instantiate<sensor_msgs::CompressedImage>();
      if (image_msg != NULL) {
	CompressedImageCallback(slam_frontend, *image_msg);
      }
    }
    {
      nav_msgs::OdometryPtr odom_msg =
          message.instantiate<nav_msgs::Odometry>();
      if (odom_msg != NULL) {
        OdometryCallback(*odom_msg);
      }
    }
  }
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  if (FLAGS_input == "") {
    fprintf(stderr, "ERROR: Must specify input file!\n");
    return 1;
  }

  // Initialize ROS.
  ros::init(argc, argv, "slam_frontend");
  ros::NodeHandle n;

  ProcessBagfile(FLAGS_input.c_str(), &n);
  return 0;
}
