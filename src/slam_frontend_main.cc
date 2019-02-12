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

#include "slam_frontend.h"
#include "slam_to_ros.h"

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

static nav_msgs::Odometry last_slam_odom;
static bool slam_needed;
static uint64_t image_num = 0;

void CompressedImageCallback(slam::Frontend& slam_frontend,
                             sensor_msgs::CompressedImage& msg) {
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
  if (slam_needed) {
    LOG(INFO) << "Observing image " << image_num << " for SLAM vision data"
        << std::endl;
    slam_frontend.ObserveImage(image, image_time, image_num, last_slam_odom);
    if (FLAGS_visualize) {
      cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
      cv::imshow("Display Image", image);
      if (cv::waitKey(16) == 27) {
        exit(0);
      }
    }
    slam_needed = false;
  }
  image_num++;
}

bool OdomCheck(const nav_msgs::Odometry& new_odom) {
    geometry_msgs::Point p;
    p.x = new_odom.pose.pose.position.x - last_slam_odom.pose.pose.position.x;
    p.y = new_odom.pose.pose.position.y - last_slam_odom.pose.pose.position.y;
    p.z = new_odom.pose.pose.position.z - last_slam_odom.pose.pose.position.z;
    if (sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2)) >= 0.1) {
      return true;
    }
    geometry_msgs::Quaternion q_old = last_slam_odom.pose.pose.orientation;
    geometry_msgs::Quaternion q_new = last_slam_odom.pose.pose.orientation;
    double inner_product = q_new.x * q_old.x + q_new.y * q_old.y + q_new.z * q_old.z + q_new.w * q_old.w;
    double angle_change = acos(2 * pow(inner_product, 2) - 1);
    if (angle_change >= 10) {
      return true;
    }
    return false;
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  if (OdomCheck(msg)) {
    last_slam_odom = msg;
    slam_needed = true;
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
  vector<string> topics;
  topics.push_back(FLAGS_image_topic.c_str());
  topics.push_back(FLAGS_odom_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  slam::Frontend slam_frontend(*n, "");
  last_slam_odom.pose.pose.position = geometry_msgs::Point();
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
  // Publish slam data
  LOG(INFO) << "Publishing SLAM data" << std::endl;
  ros::Rate LoopRate(100);
  ros::Publisher node_pub =
      n->advertise<vision_slam_frontend::SLAMNode>("slam_nodes", 1000);
  ros::Publisher corr_pub =
      n->advertise<vision_slam_frontend::VisionCorrespondence>("slam_corr", 1000);
  std::vector<slam_types::SLAMNode> nodes = slam_frontend.getSLAMNodes();
  std::vector<slam_types::VisionCorrespondence> corrs = slam_frontend.getCorrespondences();
  for (auto node : nodes) {
    node_pub.publish<>(SLAMNodeToRos(node));
  }
  for (auto corr : corrs) {
    corr_pub.publish<>(VisionCorrespondenceToRos(corr));
  }
  LOG(INFO) << "Finished publishing SLAM data" << std::endl;
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
