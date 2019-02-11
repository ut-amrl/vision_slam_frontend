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
\file    slam_frontend.cc
\brief   A vision SLAM validation framework
\author  John Bachman, (C) 2019
*/
//========================================================================

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>
#include <glog/logging.h>
#include <sstream>

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

#include "vision_slam_frontend/Keypoints.h"
#include "vision_slam_frontend/Keypoint.h"
#include "vision_slam_frontend/Matches.h"
#include "vision_slam_frontend/Match.h"

using ros::Time;
using std::string;
using std::vector;
using Eigen::Quaternionf;
using Eigen::Vector3f;

DEFINE_string(image_input, "../data/outdoor.bag", "Name of original bag file");
DEFINE_string(frontend_input, "../data/recorded_bags/testrun.bag", "Name of frontend output bag");
DEFINE_string(image_topic, "camera", "Name of ROS topic for image data");
DEFINE_string(matches_topic, "/frontend_matches", "Name of ROS topic for matches");
DEFINE_string(keypoints_topic, "/frontend_keypoints", "Name of ROS topic for keypoints");
DEFINE_bool(visualize, true, "Display images loaded");
DECLARE_string(helpon);
DECLARE_int32(v);

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);
  // Initialize ROS.
  ros::init(argc, argv, "slam_frontend");
  ros::NodeHandle n;
  
  rosbag::Bag frontend_bag;
  try {
    frontend_bag.open(FLAGS_frontend_input.c_str(), rosbag::bagmode::Read);
  } catch(rosbag::BagException exception) {
    printf("Unable to read bagfiles, reason:\n %s\n", exception.what());
    return 1;
  }
  
  std::vector<string> frontend_topics;
  frontend_topics.push_back(FLAGS_keypoints_topic.c_str());
  frontend_topics.push_back(FLAGS_matches_topic.c_str());
  rosbag::View frontend_view(frontend_bag, rosbag::TopicQuery(frontend_topics));
  
  uint64_t curr_frame  = 0;
  cv::Mat image;
  std::vector<vision_slam_frontend::Match> matches;
  // Iterate for every message.
  for (rosbag::View::iterator frontend_it = frontend_view.begin(); frontend_it != frontend_view.end(); ++frontend_it) {
    const rosbag::MessageInstance& message = *frontend_it;
    ros::spinOnce();
    {
      vision_slam_frontend::KeypointsPtr keypoints_msg =
          message.instantiate<vision_slam_frontend::Keypoints>();
      if (keypoints_msg != NULL) {
	curr_frame = keypoints_msg->pose_id;
	std::stringstream ss;
	ss << "data/raw_images/image" 
	    << std::setfill('0') << std::setw(8) << keypoints_msg->pose_id << ".jpg";
	std::cout << "Loading: " << ss.str() << std::endl;
	image = cv::imread(ss.str());
	ss.str("");
	ss << "Frame: " << keypoints_msg->pose_id;
	cv::addText(image, ss.str(), cv::Point2f(0, 20), cv::fontQt("Times", -1, CV_RGB(255, 0, 0)));
	for (auto m: matches) {
	  vision_slam_frontend::Keypoint p = keypoints_msg->points[m.found_index];
	  cv::circle(image, cv::Point2f(p.x, p.y), 5, CV_RGB(0, 255, 0));
	}
	cv::imshow("Display", image);
	if (cv::waitKey() == 27) {
	  return 0;
	}
      }
    }
    {
      vision_slam_frontend::MatchesPtr matches_msg =
	  message.instantiate<vision_slam_frontend::Matches>();
      if (matches_msg != NULL) {
	matches = matches_msg->matches;
      }
    }
  }

  return 0;
}
