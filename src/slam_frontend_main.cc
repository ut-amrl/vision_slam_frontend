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

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
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
#include "vision_slam_frontend/SLAMNode.h"
#include "vision_slam_frontend/VisionCorrespondence.h"
#include "cv_bridge/cv_bridge.h"

#include "slam_frontend.h"
#include "slam_to_ros.h"

using ros::Time;
using slam::Frontend;
using std::string;
using std::vector;
using Eigen::Quaternionf;
using Eigen::Vector3f;

DEFINE_string(image_topic,
              "/camera_right/image_raw/compressed",
              "Name of ROS topic for image data");
DEFINE_string(odom_topic,
              "/odometry/filtered",
              "Name of ROS topic for odometry data");
DEFINE_string(input, "", "Name of ROS bag file to load");
DEFINE_string(output, "", "Name of ROS bag file to output");
DEFINE_bool(visualize, false, "Display images loaded");
DECLARE_string(helpon);
DECLARE_int32(v);

void CompressedImageCallback(sensor_msgs::CompressedImage& msg,
                             Frontend* frontend) {
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
  frontend->ObserveImage(image, image_time);
  if (FLAGS_visualize) {
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    if (cv::waitKey(16) == 27) {
      exit(0);
    }
  }
}

void OdometryCallback(const nav_msgs::Odometry& msg,
                      Frontend* frontend) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector3f odom_loc(msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z);
  const Quaternionf  odom_angle(msg.pose.pose.orientation.w,
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z);
  frontend->ObserveOdometry(odom_loc, odom_angle, msg.header.stamp.toSec());
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
  vector<string> topics;
  topics.push_back(FLAGS_image_topic.c_str());
  topics.push_back(FLAGS_odom_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  slam::Frontend slam_frontend("");
  double bag_t_start = -1;
  // Iterate for every message.
  for (rosbag::View::iterator it = view.begin();
       ros::ok() && it != view.end();
       ++it) {
    const rosbag::MessageInstance& message = *it;
    if (FLAGS_v > 1) {
      printf("Message t: %f\n", message.getTime().toSec());
    }
    if (bag_t_start < 0.0) {
      bag_t_start = message.getTime().toSec();
    }
    ros::spinOnce();
    {
      sensor_msgs::CompressedImagePtr image_msg =
          message.instantiate<sensor_msgs::CompressedImage>();
      if (image_msg != NULL) {
        CompressedImageCallback(*image_msg, &slam_frontend);
      }
    }
    {
      nav_msgs::OdometryPtr odom_msg =
          message.instantiate<nav_msgs::Odometry>();
      if (odom_msg != NULL) {
        OdometryCallback(*odom_msg, &slam_frontend);
      }
    }
  }
  printf("Done processing bag file.\n");
  // Publish slam data
  if (FLAGS_output != "") {
    rosbag::Bag output_bag;
    try {
      output_bag.open(FLAGS_output.c_str(), rosbag::bagmode::Write);
    } catch(rosbag::BagException exception) {
      printf("Unable to open %s, reason:\n %s\n",
             FLAGS_output.c_str(),
             exception.what());
      return;
    }
    std::vector<slam_types::SLAMNode> nodes = slam_frontend.getSLAMNodes();
    std::vector<slam_types::VisionCorrespondence> corrs = slam_frontend.getCorrespondences();
    std::vector<cv::Mat> debug_images = slam_frontend.getDebugImages();
    for (auto node : nodes) {
      output_bag.write<vision_slam_frontend::SLAMNode>("slam_nodes",
                                                       ros::Time::now(),
                                                       SLAMNodeToRos(node));
    }
    for (auto corr : corrs) {
      output_bag.write<vision_slam_frontend::VisionCorrespondence>("slam_corrs",
                                                                   ros::Time::now(),
                                                                   VisionCorrespondenceToRos(corr));
    }
    cv_bridge::CvImage img_tranform;
    uint64_t count = 0;
    for (auto im: debug_images) {
      std_msgs::Header h;
      h.seq = count++;
      h.stamp = ros::Time::now();
      img_tranform.image = im;
      img_tranform.encoding = sensor_msgs::image_encodings::RGB8;
      output_bag.write<sensor_msgs::Image>("slam_debug_images",
                                           ros::Time::now(),
                                           img_tranform.toImageMsg());
    }
    output_bag.close();
  }
}

void SignalHandler(int signum) {
  printf("Exiting with signal %d\n", signum);
  exit(0);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  if (FLAGS_input == "") {
    fprintf(stderr, "ERROR: Must specify input file!\n");
    return 1;
  }
  if (FLAGS_output == "") {
    fprintf(stderr, "ERROR: Must specify output file!\n");
    return 1;
  }
  // Initialize ROS.
  ros::init(argc, argv, "slam_frontend");
  ros::NodeHandle n;
  signal(SIGINT, SignalHandler);
  ProcessBagfile(FLAGS_input.c_str(), &n);
  return 0;
}
