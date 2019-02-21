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

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "opencv2/opencv.hpp"

#include "slam_types.h"

namespace slam {
/* A container for slam configuration data */
struct FrontendConfig {
 public:
  enum class DescriptorExtractorType {
    AKAZE,
    ORB,
    BRISK,
    SURF,
    SIFT,
    FREAK
  };
  FrontendConfig();
  void Load(const std::string& path);

  bool debug_images_;
  DescriptorExtractorType  descriptor_extract_type_;
  float best_percent_;
  float nn_match_ratio_;
  // If the robot translates by this much, as reported by odometry, the next
  // image will be used for a vision frame.
  float min_odom_translation;
  // If the robot rotates by this much, as reported by odometry, the next
  // image will be used for a vision frame.
  float min_odom_rotation;
  // The minimum number of feature matches that must exist between a pair of
  // frames to add a vision correspondence feature.
  uint32_t min_vision_matches;
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
  Frame() {}
  std::vector<cv::DMatch> GetMatches(const slam::Frame& frame,
                                     double nn_match_ratio);
  uint64_t frame_ID_;
  cv::Ptr<cv::BFMatcher> matcher_;
  std::vector<cv::KeyPoint> keypoints_;
  std::vector<bool> is_initial_;
  std::vector<int64_t> initial_ids_;
  cv::Mat descriptors_;
  FrontendConfig config_;
  std::unordered_map<uint64_t,
                     std::pair<uint64_t, uint64_t>> initial_appearances;
  cv::Mat debug_image_;
};

/* The actual processing unit for the entire frontend */
class Frontend {
 public:
  explicit Frontend(const std::string& config_path);
  // Observe a new image. Extract features, and match to past frames.
  void ObserveImage(const cv::Mat& image,
                    double time);
  // Observe new odometry message.
  void ObserveOdometry(const Eigen::Vector3f& translation,
                       const Eigen::Quaternionf& rotation,
                       double timestamp);
  std::vector<cv::Mat> getDebugImages();
  // Return the latest debug image.
  cv::Mat GetLastDebugImage();

  // Get a fully instantiated SLAM problem with the data collected so far.
  void GetSLAMProblem(slam_types::SLAMProblem* problem) const;

 private:
  // Returns true iff odometry reports that the robot has moved sufficiently to
  // warrant a vision update.
  bool OdomCheck();
  // Extract feature descriptors from image, and construct a Vision frame.
  void ExtractFeatures(cv::Mat image, Frame* curr_frame);
  // Return feature matches between frame1 and frame2, and also update frame2
  // to track the initial frame for all matches.
  void GetFeatureMatches(Frame* frame1,
                         Frame* frame2,
                         slam_types::VisionFactor* correspondence);
  // Create a new odometry factor, and reset odometry tracking variables.
  void AddOdometryFactor();

  // Indicates if odometry has been initialized or not.
  bool odom_initialized_;
  // Initial odometry translation.
  Eigen::Vector3f init_odom_translation_;
  // Initial odometry rotation.
  Eigen::Quaternionf init_odom_rotation_;
  // Previous odometry-reported pose translation.
  Eigen::Vector3f prev_odom_translation_;
  // Previous odometry-reported pose rotation.
  Eigen::Quaternionf prev_odom_rotation_;
  // Latest odometry-reported pose translation.
  Eigen::Vector3f odom_translation_;
  // Latest odometry-reported pose rotation.
  Eigen::Quaternionf odom_rotation_;
  // Latest odometry timestamp.
  double odom_timestamp_;
  // Configuration parameters for frontend.
  FrontendConfig config_;
  // Next frame ID.
  uint64_t curr_frame_ID_;
  std::vector<Frame> frame_list_;
  cv::Ptr<cv::Feature2D> descriptor_extractor_;
  cv::Ptr<cv::FastFeatureDetector> fast_feature_detector_;
  // Formatted data for slam problem
  std::vector<slam_types::VisionFactor> vision_factors_;
  std::vector<slam_types::SLAMNode> nodes_;
  std::vector<slam_types::OdometryFactor> odometry_factors_;
  // Debug
  std::vector<cv::Mat> debug_images_;
};
}  // namespace slam

#endif   // __SLAM_FRONTEND_H__
