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
\file    slam_frontend.h
\brief   A vision SLAM frontend
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#ifndef __SLAM_FRONTEND_H__
#define __SLAM_FRONTEND_H__

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "slam_types.h"

namespace slam {
/* A container for slam configuration data */
class FrontendConfig {
 public:
  enum DescriptorExtractorType {
    AKAZE, 
    ORB,
    BRISK,
    SURF,
    SIFT,
    FREAK
  };
  FrontendConfig();
  void Load(const std::string& path);
  DescriptorExtractorType getDescExType() { return descriptor_extract_type_; }
  double getBestPercent() { return best_percent_; }
  double getNNMatchRatio() { return nn_match_ratio_; }
  uint32_t getMaxFrameLife() { return frame_life_; }
  cv::NormTypes getBFMatcherParam() { return bf_matcher_param_; }
  void setBFMatcherParam(cv::NormTypes bf_matcher_param) { bf_matcher_param_ = 
bf_matcher_param; }
 private:
  DescriptorExtractorType  descriptor_extract_type_;
  double best_percent_;
  double nn_match_ratio_;
  uint32_t frame_life_;
  cv::NormTypes bf_matcher_param_;
};

/* A container for slam node data */
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
  std::unordered_map<uint64_t, 
                     std::pair<uint64_t, uint64_t>> initial_appearances;
};

/* The actual processing unit for the entire frontend */
class Frontend {
 public:
  Frontend(ros::NodeHandle& n, const std::string& config_path);
  // Observe a new image. Extract features, and match to past frames.
  void ObserveImage(const cv::Mat& image, 
                    double time,
                    const nav_msgs::Odometry& odom_msg);
  void ObserveOdometry(const Eigen::Vector3f& translation,
                       const Eigen::Quaternionf& rotation,
                       double time);
  std::vector<slam_types::VisionCorrespondence> getCorrespondences();
  std::vector<slam_types::SLAMNode> getSLAMNodes();
 private:
  slam_types::VisionCorrespondencePair CreateVisionPair(uint64_t pose_i_idx,
                                                        uint64_t pose_j_idx,
                                                        uint64_t pose_initial,
                                                        uint64_t pose_initial_idx);
  slam_types::VisionCorrespondence CreateVisionCorrespondence(uint64_t pose_i,
                                                               uint64_t pose_j,
                                                               const std::vector<slam_types::VisionCorrespondencePair> pairs);
  slam_types::VisionFeature CreateVisionFeature(uint64_t id, cv::Point2f pixel);
  slam_types::SLAMNode CreateSLAMNode(uint64_t pose_i,
                                      const std::vector<slam_types::VisionFeature>& features,
                                      const nav_msgs::Odometry& odom_msg);
  FrontendConfig config_;
  nav_msgs::Odometry last_slam_odom_;
  cv::NormTypes bf_matcher_param_;
  std::vector<Frame> frame_list_;
  cv::Ptr<cv::Feature2D> descriptor_extractor_;
  cv::Ptr<cv::FastFeatureDetector> fast_feature_detector_;
  // Formatted data for slam problem
  std::vector<slam_types::VisionCorrespondence> correspondences_;
  std::vector<slam_types::SLAMNode> nodes_;
  uint64_t curr_frame_ID_ = 0;
};
}  // namespace slam

#endif   // __SLAM_FRONTEND_H__
