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

#include "slam_frontend_backend_comm.h"

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "vision_slam_frontend/Match.h"
#include "vision_slam_frontend/Matches.h"
#include "vision_slam_frontend/Keypoint.h"
#include "vision_slam_frontend/Keypoints.h"

slam::FrontendBackendComm::FrontendBackendComm(ros::NodeHandle& n) {
  ros::Rate LoopRate(1000);
  match_pub_ = n.advertise<vision_slam_frontend::Matches>("frontend_matches", 2000);
  keypoint_pub_ = n.advertise<vision_slam_frontend::Keypoints>("frontend_keypoints", 2000);
  curr_matches_ = new vision_slam_frontend::Matches();
  curr_keypoints_ = new vision_slam_frontend::Keypoints();
}

void slam::FrontendBackendComm::QueueMatch(uint64_t initial_frame, 
					   uint64_t initial_index, 
					   uint64_t found_index) {
  vision_slam_frontend::Match m;
  m.pose_initial = initial_frame;
  m.initial_index = initial_index;
  m.found_index = found_index;
  curr_matches_->matches.push_back(m);
}

void slam::FrontendBackendComm::BroadcastMatches(uint64_t old_frame, uint64_t new_frame) {
  curr_matches_->past_frame = old_frame;
  curr_matches_->found_frame = new_frame;
  match_pub_.publish<vision_slam_frontend::Matches>(*curr_matches_);
  free(curr_matches_);
  curr_matches_ = new vision_slam_frontend::Matches();
}

void slam::FrontendBackendComm::QueueKeypoint(cv::KeyPoint kpt) {
  vision_slam_frontend::Keypoint k;
  k.x = kpt.pt.x;
  k.y = kpt.pt.y;
  curr_keypoints_->points.push_back(k);
}

void slam::FrontendBackendComm::BroadcastKeypoints(uint64_t pose_id, 
						   const std::vector<cv::KeyPoint> keypoints) {
  curr_keypoints_->pose_id = pose_id;
  keypoint_pub_.publish<vision_slam_frontend::Keypoints>(*curr_keypoints_);
  free(curr_keypoints_);
  curr_keypoints_ = new vision_slam_frontend::Keypoints();
}
