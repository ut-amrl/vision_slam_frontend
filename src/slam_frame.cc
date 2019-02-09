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
\brief   A vision SLAM Frame
\author  John Bachman, (C) 2019
*/
//========================================================================

#include "opencv2/opencv.hpp"
#include <opencv2/xfeatures2d.hpp>
#include "slam_frame.h"

slam::Frame::Frame(const std::vector<cv::KeyPoint>& keypoints, 
		   const cv::Mat& descriptors,
		   const slam::FrontendConfig& config,
		   uint64_t frame_ID) {
  keypoints_ = keypoints;
  descriptors_ = descriptors;
  config_ = config;
  frame_ID_ = frame_ID;
  matcher_ = cv::BFMatcher::create(config_.getBFMatcherParam());
}

std::vector<cv::DMatch> slam::Frame::GetMatches(const slam::Frame& frame, 
					        double nn_match_ratio) {
  std::vector<std::vector<cv::DMatch>> matches;
  matcher_->knnMatch(descriptors_, frame.descriptors_, matches, 2);
  std::vector<cv::DMatch> best_matches;
  for (size_t i = 0; i < matches.size(); i++) {    
    cv::DMatch first = matches[i][0];
    float dist1 = matches[i][0].distance;
    float dist2 = matches[i][1].distance;
    if(dist1 < config_.getNNMatchRatio() * dist2) {
	best_matches.push_back(first);
    }
  }
  return best_matches;
}

std::pair<uint64_t, uint64_t> slam::Frame::GetInitialFrame(cv::DMatch match) {
  auto result = initial_appearances.find(match.trainIdx);
  if (result == initial_appearances.end()) {
    //We didn't find it in the database, this must be the first time then.
    auto first_app = std::pair<uint64_t, uint64_t>(frame_ID_, match.trainIdx);
    initial_appearances.insert({match.trainIdx, first_app});
    return first_app;
  }
  return result->second;
}

void slam::Frame::AddMatchInitial(cv::DMatch match, 
				  std::pair<uint64_t, uint64_t> initial) {
  initial_appearances.insert({match.queryIdx, initial});
}
