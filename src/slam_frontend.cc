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
\brief   A vision SLAM frontend
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <stdlib.h>
#include <algorithm>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <glog/logging.h>

#include "slam_frontend.h"
#include "vio_types.h"
#include "slam_frontend_backend_comm.h"

using std::string;
using std::vector;
using Eigen::Quaternionf;
using Eigen::Vector3f;

slam::Frontend::Frontend(ros::NodeHandle& n, const std::string& config_path) {
  fast_feature_detector_ = cv::FastFeatureDetector::create(10, true);
  broadcaster_ = new slam::FrontendBackendComm(n);
  switch(config_.getDescExType()) {
    case FrontendConfig::DescriptorExtractorType::AKAZE:
      descriptor_extractor_ = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0001f, 10, 5, cv::KAZE::DIFF_PM_G2);
      bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    case FrontendConfig::DescriptorExtractorType::ORB:
      descriptor_extractor_ = cv::ORB::create(10000, 1.04f, 50, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
      bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    case FrontendConfig::DescriptorExtractorType::BRISK:
      descriptor_extractor_ = cv::BRISK::create(20, 7, 1.1f);
      bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    case FrontendConfig::DescriptorExtractorType::SURF:
      descriptor_extractor_ = cv::xfeatures2d::SURF::create();
      bf_matcher_param_ = cv::NORM_L2;
      break;
    case FrontendConfig::DescriptorExtractorType::SIFT:
      descriptor_extractor_ = cv::xfeatures2d::SIFT::create();
      bf_matcher_param_ = cv::NORM_L2;
      break;
    case FrontendConfig::DescriptorExtractorType::FREAK:
      descriptor_extractor_ = cv::xfeatures2d::FREAK::create(false, true, 40.0f, 20);
      bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    default:
      LOG(ERROR) << "Could not recognize descriptor extractor option.";
      exit(1);   
  }
}

void slam::Frontend::ObserveImage(const cv::Mat& image, double time) {
  std::vector<cv::KeyPoint> frame_keypoints;
  cv::Mat frame_descriptors;
  if (config_.getDescExType() == FrontendConfig::DescriptorExtractorType::FREAK) {
    fast_feature_detector_->detect(image, frame_keypoints);
    descriptor_extractor_->compute(image, frame_keypoints, frame_descriptors);
  } else {
    descriptor_extractor_->detectAndCompute(image, 
					    cv::noArray(), 
					    frame_keypoints, 
					    frame_descriptors);
  }
  Frame curr_frame(frame_keypoints,
		   frame_descriptors, 
		   config_, 
		   frame_ID_count_++);
  for(uint32_t frame_num = 0; frame_num < frame_list_.size(); frame_num++) {
    Frame& past_frame = frame_list_[frame_num];
    std::vector<cv::DMatch> matches = 
	curr_frame.GetMatches(past_frame, config_.getNNMatchRatio());
    std::sort(matches.begin(), matches.end());
    const int num_good_matches = matches.size() * config_.getBestPercent();
    matches.erase(matches.begin() + num_good_matches, matches.end());
    for (auto match: matches) {
      std::pair<uint64_t, uint64_t> initial = 
	  past_frame.GetInitialFrame(match);
      //Add it to the original frame.
      curr_frame.AddMatchInitial(match, initial);
      broadcaster_->QueueMatch(initial.first,
			       initial.second,
			       match.queryIdx);
    }
    broadcaster_->BroadcastMatches(past_frame.frame_ID_, curr_frame.frame_ID_);
  }
  for (auto k: curr_frame.keypoints_) {
    broadcaster_->QueueKeypoint(k);
  }
  broadcaster_->BroadcastKeypoints(curr_frame.frame_ID_, curr_frame.keypoints_);
  if (frame_list_.size() >= config_.getMaxFrameLife()) {
    frame_list_.erase(frame_list_.begin());
  }
  frame_list_.push_back(curr_frame);
}