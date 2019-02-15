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
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"

#include "slam_frontend.h"

using std::string;
using std::vector;
using Eigen::Quaternionf;
using Eigen::Vector3f;

/* --- Frontend Implementation Code --- */

bool OdomCheck(const nav_msgs::Odometry& last_slam_odom, nav_msgs::Odometry& new_odom) {
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

slam::Frontend::Frontend(ros::NodeHandle& n, const std::string& config_path) {
  last_slam_odom_.pose.pose.position = geometry_msgs::Point();
  fast_feature_detector_ = cv::FastFeatureDetector::create(10, true);
  switch(config_.getDescExType()) {
    case FrontendConfig::DescriptorExtractorType::AKAZE:
      descriptor_extractor_ = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB,
                                                0, 3, 0.0001f, 10, 5,
                                                cv::KAZE::DIFF_PM_G2);
      bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    case FrontendConfig::DescriptorExtractorType::ORB:
      descriptor_extractor_ = cv::ORB::create(10000, 1.04f, 50, 31, 0, 2,
                                              cv::ORB::HARRIS_SCORE,
                                              31, 20);
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
      descriptor_extractor_ = cv::xfeatures2d::FREAK::create(false, true,
                                                             40.0f, 20);
      bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    default:
      LOG(ERROR) << "Could not recognize descriptor extractor option.";
      exit(1);
  }
}

void slam::Frontend::ObserveImage(const cv::Mat& image,
                                  double time,
                                  const nav_msgs::Odometry& odom_msg) {
  // Check from the odometry if its time to run SLAM
  if(!OdomCheck(odom_msg, last_slam_odom_)) {
    curr_frame_ID_++;
    return;
  } else {
    curr_frame_ID_++;
    last_slam_odom_ = odom_msg;
  }
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
                   curr_frame_ID_);
  for(uint32_t frame_num = 0; frame_num < frame_list_.size(); frame_num++) {
    std::vector<slam_types::VisionCorrespondencePair> pairs;
    Frame& past_frame = frame_list_[frame_num];
    std::vector<cv::DMatch> matches = 
        curr_frame.GetMatches(past_frame, config_.getNNMatchRatio());
    std::sort(matches.begin(), matches.end());
    const int num_good_matches = matches.size() * config_.getBestPercent();
    matches.erase(matches.begin() + num_good_matches, matches.end());
    // Restructure matches, add all keypoints to new list.
    for (auto match: matches) {
      std::pair<uint64_t, uint64_t> initial = past_frame.GetInitialFrame(match);
      // Add it to the original frame.
      curr_frame.AddMatchInitial(match, initial);
      pairs.push_back(CreateVisionPair(match.trainIdx,
                                       match.queryIdx,
                                       initial.first,
                                       initial.second));
    }
    correspondences_.push_back(CreateVisionCorrespondence(past_frame.frame_ID_,
                                                         curr_frame.frame_ID_,
                                                         pairs));
  }
  std::vector<slam_types::VisionFeature> features;
  for (uint64_t i = 0; i < curr_frame.keypoints_.size(); i++) {
    features.push_back(CreateVisionFeature(i, curr_frame.keypoints_[i].pt));
  }
  nodes_.push_back(CreateSLAMNode(curr_frame.frame_ID_, features, odom_msg));
  if (frame_list_.size() >= config_.getMaxFrameLife()) {
    frame_list_.erase(frame_list_.begin());
  }
  frame_list_.push_back(curr_frame);
}

std::vector<slam_types::VisionCorrespondence>
slam::Frontend::getCorrespondences() {
  return correspondences_;
}

std::vector<slam_types::SLAMNode>
slam::Frontend::getSLAMNodes() {
  return nodes_;
}

slam_types::VisionCorrespondencePair
slam::Frontend::CreateVisionPair(uint64_t pose_i_idx,
                                 uint64_t pose_j_idx,
                                 uint64_t pose_initial,
                                 uint64_t pose_initial_idx) {
  slam_types::VisionCorrespondencePair pair;
  pair.pose_i_id = pose_i_idx;
  pair.pose_j_id = pose_j_idx;
  pair.pose_initial = pose_initial;
  pair.pose_initial_id = pose_initial_idx;
  return pair;
}

slam_types::VisionCorrespondence
slam::Frontend::CreateVisionCorrespondence(uint64_t pose_i,
                                           uint64_t pose_j,
                                           const std::vector<slam_types::VisionCorrespondencePair> pairs) {
  slam_types::VisionCorrespondence corr;
  corr.pose_i = pose_i;
  corr.pose_j = pose_j;
  corr.feature_matches = pairs;
  return corr;
}

slam_types::VisionFeature
slam::Frontend::CreateVisionFeature(uint64_t id, cv::Point2f pixel) {
  slam_types::VisionFeature vis;
  vis.id = id;
  vis.descriptor_id = 0;
  vis.pixel = Vector2f(pixel.x, pixel.y);
  return vis;
}

slam_types::SLAMNode
slam::Frontend::CreateSLAMNode(uint64_t pose_i,
                                     const std::vector<slam_types::VisionFeature>& features,
                                     const nav_msgs::Odometry& odom_msg) {
  slam_types::SLAMNode node;
  node.id = pose_i;
  node.is_vision_node = true;
  node.features = features;
  //node.pose = odom_msg.pose.pose;
  return node;
}

/* --- Frame Implementation Code --- */

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

/* --- Config Implementation Code --- */

slam::FrontendConfig::FrontendConfig() {
  //Load Default values
  descriptor_extract_type_ = FrontendConfig::DescriptorExtractorType::AKAZE;
  best_percent_ = 0.3f;
  nn_match_ratio_ = 0.8f;
  frame_life_ = 5;
  bf_matcher_param_ = cv::NORM_HAMMING;
}
