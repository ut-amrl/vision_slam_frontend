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

// Pinhole camera intrinsics parameters.
// The convention used here matches that of OpenCV:
// https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/
//     camera_calibration.html
struct CameraIntrinsics {
  // Radial distortion parameters.
  float k1, k2, k3;
  // Tangential distortion parameters.
  float p1, p2;
  // Focal length x.
  float fx;
  // Focal length y.
  float fy;
  // Principal point x.
  float cx;
  // Principal point y.
  float cy;
};

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
  // The maximum feature track length -- the maximum number of pose-steps older
  // than the current pose that should be compared for vision factors.
  uint32_t frame_life_;
  // Feature matching norm type (e.g. L2, Hamming, etc.).
  cv::NormTypes bf_matcher_param_;
  // Camera intrinsics.
  CameraIntrinsics intrinsics_left, intrinsics_right;
  // Derived parameters, computed from intrinsics.
  cv::Mat camera_matrix_left, camera_matrix_right, distortion_coeffs_left,
      distortion_coeffs_right, projection_left, projection_right;
  Eigen::Matrix3f fundamental;
  // Affine transforms from frame of camera to robot.
  Eigen::Affine3f left_cam_to_robot;
};

/* A container for slam node data */
class Frame {
 public:
  Frame(const std::vector<cv::KeyPoint>& keypoints,
        const cv::Mat& descriptors,
        uint64_t frame_ID);
  Frame() {}
  uint64_t frame_ID_;
  std::vector<cv::KeyPoint> keypoints_;
  std::vector<bool> is_initial_;
  std::vector<int64_t> initial_ids_;
  cv::Mat descriptors_;
  std::unordered_map<uint64_t,
                     std::pair<uint64_t, uint64_t>> initial_appearances;
  cv::Mat debug_image_;
};

/* The actual processing unit for the entire frontend */
class Frontend {
 public:
  explicit Frontend(const std::string& config_path);
  // Observe a new image. Extract features, and match to past frames. Returns
  // true iff a new SLAM node is added to the SLAM problem.
  bool ObserveImage(const cv::Mat& left_image,
                    const cv::Mat& right_image,
                    double time);
  // Observe new odometry message.
  void ObserveOdometry(const Eigen::Vector3f& translation,
                       const Eigen::Quaternionf& rotation,
                       double timestamp);
  std::vector<cv::Mat> getDebugImages();
  // Return the latest debug image.
  cv::Mat GetLastDebugImage();
  cv::Mat GetLastDebugStereoImage();
  std::vector<cv::Mat> getDebugStereoImages();

  // Get a fully instantiated SLAM problem with the data collected so far.
  void GetSLAMProblem(slam_types::SLAMProblem* problem) const;

  // Get the number od poses (SLAM nodes) added so far.
  int GetNumPoses();

  // Get the frontend configuration parameters.
  FrontendConfig GetConfig() { return config_; }

 private:
  // Returns true iff odometry reports that the robot has moved sufficiently to
  // warrant a vision update.
  bool OdomCheck();
  // Extract feature descriptors from image, and construct a Vision frame.
  void ExtractFeatures(cv::Mat image, Frame* curr_frame);
  // Return feature matches between frame1 and frame2, and also update frame2
  // to track the initial frame for all matches.
  slam_types::VisionFactor* GetFeatureMatches(Frame* past_frame_ptr,
                                              Frame* curr_frame_ptr);
  // Returns the matches between frame_query and frame_train
  std::vector<cv::DMatch> GetMatches(const Frame& frame_query,
                                     const Frame& frame_train,
                                     double nn_match_ratio);
  void RemoveAmbigStereo(Frame* left,
                         Frame* right,
                         const std::vector<cv::DMatch> stereo_matches);
  // Create a new odometry factor, and reset odometry tracking variables.
  void AddOdometryFactor();
  // Removes radial distortion from all observed feature points.
  void UndistortFeaturePoints(std::vector<slam_types::VisionFeature>* features);
  // Takes the stereo images of the same scene and uses the corresponding
  // matches to get the depth estimate of the 3D point.
  void Calculate3DPoints(Frame* left_frame,
                         Frame* right_frame,
                         std::vector<Eigen::Vector3f>* points);

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
  // Brute-force matcher for descriptor matching
  cv::Ptr<cv::BFMatcher> matcher_;
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
  std::vector<cv::Mat> debug_stereo_images_;
};
}  // namespace slam

#endif   // __SLAM_FRONTEND_H__
