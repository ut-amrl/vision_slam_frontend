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
\brief   A vision SLAM communicator
\author  John Bachman, (C) 2019
*/
//========================================================================

#ifndef SLAM_FRONTEND_BACKEND_COMM_H
#define SLAM_FRONTEND_BACKEND_COMM_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "vision_slam_frontend/Matches.h"
#include "vision_slam_frontend/Keypoints.h"

namespace slam {
class FrontendBackendComm {
 public:
  FrontendBackendComm(ros::NodeHandle& n);
  void QueueMatch(uint64_t initial_frame, 
		  uint64_t initial_index, 
		  uint64_t found_index);
  void BroadcastMatches(uint64_t old_frame, uint64_t new_frame);
  void QueueKeypoint(cv::KeyPoint kpt);
  void BroadcastKeypoints(uint64_t pose_id, 
			  const std::vector<cv::KeyPoint> keypoints);
 private:
  ros::Publisher match_pub_;
  ros::Publisher keypoint_pub_;
  vision_slam_frontend::Matches* curr_matches_;
  vision_slam_frontend::Keypoints* curr_keypoints_;
};
}

#endif // SLAM_FRONTEND_BACKEND_COMM_H
