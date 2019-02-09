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

#ifndef __SLAM_FRAME_H__
#define __SLAM_FRAME_H__

#include <utility>

#include "opencv2/opencv.hpp"
#include <opencv2/features2d.hpp>

#include "slam_config.h"

namespace slam {
class Frame {
 public:
  Frame(const std::vector<cv::KeyPoint>& keypoints,
	const cv::Mat& descriptors,
        const slam::FrontendConfig& config,
	uint64_t frame_ID);
  std::vector<cv::DMatch> GetMatches(const slam::Frame& frame,
				     double nn_match_ratio);
  std::pair<uint64_t, uint64_t> GetInitialFrame(cv::DMatch match_idx);
  void AddMatchInitial(cv::DMatch match, 
				    std::pair<uint64_t, uint64_t> initial);
  uint64_t frame_ID_;
  cv::Ptr<cv::BFMatcher> matcher_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;
  FrontendConfig config_;
  std::unordered_map<uint64_t, std::pair<uint64_t, uint64_t>> initial_appearances;
};
} //namespace slam

#endif // SLAM_FRAME_H
