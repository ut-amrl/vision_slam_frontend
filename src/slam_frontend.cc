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
#include <string>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "glog/logging.h"

#include "slam_frontend.h"

using slam_types::RobotPose;
using slam_types::SLAMNode;
using slam_types::SLAMProblem;
using slam_types::OdometryFactor;
using slam_types::VisionFactor;
using slam_types::FeatureMatch;
using slam_types::VisionFeature;
using std::string;
using std::vector;
using Eigen::Quaternionf;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Matrix;

/* --- Frontend Implementation Code --- */

namespace {
template<typename T>
Eigen::Matrix<T, 2, 1> OpenCVToEigen(const cv::Point_<T>& p) {
  return Eigen::Matrix<T, 2, 1>(p.x, p.y);
}

template<typename T>
cv::Point_<T> EigenToOpenCV(const Eigen::Matrix<T, 2, 1>& p) {
  return cv::Point_<T>(p.x(), p.y());
}

}  // namespace
namespace slam {

void Frontend::Calculate3DLocations(
                          Frame* left_frame,
                          Frame* right_frame,
                          std::vector<Eigen::Vector3f>* locations) {
  // Convert keypoints into array of points.
  std::vector<cv::Point2f> left_points;
  std::vector<cv::Point2f> right_points;
  // The reason we pass in right frame first is because we don't want to modify
  // the left frames is_initial_ book-keeping.
  // This means right will be the 'initial' and the left_frame
  // will be the 'current' frame.
  VisionFactor matches;
  // Assure that every point has a match.
  float best_percent = config_.best_percent_;
  config_.best_percent_ = 1.0;
  GetFeatureMatches(right_frame, left_frame, &matches);
  config_.best_percent_ = best_percent;
  for (FeatureMatch match : matches.feature_matches) {
    const auto& left_pt = left_frame->keypoints_[match.id_current].pt;
    const auto& right_pt = right_frame->keypoints_[match.id_initial].pt;
    // if (fabs(left_pt.y - right_pt.y) > 5) continue;
    right_points.push_back(right_pt);
    left_points.push_back(left_pt);
    if (false) {
      printf("[%6.1f %6.1f] [%6.1f %6.1f]\n",
            left_pt.x,
            left_pt.y,
            right_pt.x,
            right_pt.y);
    }
  }
  cv::Mat triangulated_points;
  if (true) {
    printf("TODO: triangulatePoints requires a rectified stereo. See:\n\n"
           "https://docs.opencv.org/2.4/modules/calib3d/doc/"
           "camera_calibration_and_3d_reconstruction.html#stereocalibrate\n"
           "and:\n"
           "https://docs.opencv.org/2.4/modules/calib3d/doc/"
           "camera_calibration_and_3d_reconstruction.html#triangulatepoints\n");
    exit(0);
  }
  cv::triangulatePoints(config_.projection_left,
                        config_.projection_right,
                        left_points,
                        right_points,
                        triangulated_points);
  // Make sure all keypoints are matched to something.
  CHECK_EQ(triangulated_points.cols, matches.feature_matches.size());
  for (int64_t c = 0; c < triangulated_points.cols; ++c) {
    auto col = triangulated_points.col(c);
    auto vec = Eigen::Vector3f(col.ptr<float>()[0],
                               col.ptr<float>()[1],
                               col.ptr<float>()[2]);
    locations->push_back(vec);
  }
}

cv::Mat CreateDebugImage(const cv::Mat& image,
                         const Frame& frame_initial,
                         const Frame& frame_current,
                         const VisionFactor& corr) {
  cv::Mat return_image = image.clone();
  cv::cvtColor(return_image, return_image, cv::COLOR_GRAY2RGB);
  for (const slam_types::FeatureMatch c : corr.feature_matches) {
    cv::circle(return_image,
               frame_initial.keypoints_[c.id_initial].pt,
               5, CV_RGB(255, 0, 0));
    cv::line(return_image,
             frame_initial.keypoints_[c.id_initial].pt,
             frame_current.keypoints_[c.id_current].pt,
             CV_RGB(0, 255, 0));
  }
  return return_image;
}

bool Frontend::OdomCheck() {
  if (!odom_initialized_) return false;
  if ((prev_odom_translation_ - odom_translation_).norm() >
      config_.min_odom_translation) {
    return true;
  }
  if (prev_odom_rotation_.angularDistance(odom_rotation_) >
      config_.min_odom_rotation) {
    return true;
    }
  return false;
}

Frontend::Frontend(const string& config_path) :
    odom_initialized_(false),
    curr_frame_ID_(0) {
  fast_feature_detector_ = cv::FastFeatureDetector::create(10, true);
  switch (config_.descriptor_extract_type_) {
    case FrontendConfig::DescriptorExtractorType::AKAZE: {
      descriptor_extractor_ = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB,
                                                0,
                                                3,
                                                0.0001f,
                                                10,
                                                5,
                                                cv::KAZE::DIFF_PM_G2);
      config_.bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    }
    case FrontendConfig::DescriptorExtractorType::ORB: {
      descriptor_extractor_ = cv::ORB::create(10000,
                                              1.04f,
                                              50,
                                              31,
                                              0,
                                              2,
                                              cv::ORB::HARRIS_SCORE,
                                              31,
                                              20);
      config_.bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    }
    case FrontendConfig::DescriptorExtractorType::BRISK: {
      descriptor_extractor_ = cv::BRISK::create(20, 7, 1.1f);
      config_.bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    }
    case FrontendConfig::DescriptorExtractorType::SURF: {
      descriptor_extractor_ = cv::xfeatures2d::SURF::create();
      config_.bf_matcher_param_ = cv::NORM_L2;
      break;
    }
    case FrontendConfig::DescriptorExtractorType::SIFT: {
      descriptor_extractor_ = cv::xfeatures2d::SIFT::create();
      config_.bf_matcher_param_ = cv::NORM_L2;
      break;
    }
    case FrontendConfig::DescriptorExtractorType::FREAK: {
      descriptor_extractor_ = cv::xfeatures2d::FREAK::create(false,
                                                             true,
                                                             40.0f,
                                                             20);
      config_.bf_matcher_param_ = cv::NORM_HAMMING;
      break;
    }
    default: {
      LOG(ERROR) << "Could not recognize descriptor extractor option.";
      exit(1);
    }
  }
}

void Frontend::ObserveOdometry(const Vector3f& translation,
                               const Quaternionf& rotation,
                               double timestamp) {
  odom_translation_ = translation;
  odom_rotation_ = rotation;
  odom_timestamp_ = timestamp;
  if (!odom_initialized_) {
    init_odom_rotation_ = rotation;
    init_odom_translation_ = translation;
    prev_odom_rotation_ = odom_rotation_;
    prev_odom_translation_ = odom_translation_;
    odom_initialized_ = true;
  }
}

void Frontend::ExtractFeatures(cv::Mat image, Frame* frame) {
  vector<cv::KeyPoint> frame_keypoints;
  cv::Mat frame_descriptors;
  if (config_.descriptor_extract_type_ ==
      FrontendConfig::DescriptorExtractorType::FREAK) {
    fast_feature_detector_->detect(image, frame_keypoints);
    descriptor_extractor_->compute(image, frame_keypoints, frame_descriptors);
  } else {
    descriptor_extractor_->detectAndCompute(image,
                                            cv::noArray(),
                                            frame_keypoints,
                                            frame_descriptors);
  }
  *frame = Frame(frame_keypoints, frame_descriptors, config_, curr_frame_ID_);
}

void Frontend::GetFeatureMatches(Frame* past_frame_ptr,
                                 Frame* curr_frame_ptr,
                                 VisionFactor* correspondence) {
  Frame& past_frame = *past_frame_ptr;
  Frame& curr_frame = *curr_frame_ptr;
  vector<FeatureMatch> pairs;
  vector<cv::DMatch> matches =
        past_frame.GetMatches(curr_frame, config_.nn_match_ratio_);
  std::sort(matches.begin(), matches.end());
  const int num_good_matches = matches.size() * config_.best_percent_;
  matches.erase(matches.begin() + num_good_matches, matches.end());
  // Restructure matches, add all keypoints to new list.
  for (auto match : matches) {
    // Add it to vision factor.
    pairs.push_back(FeatureMatch(match.queryIdx,
                                 match.trainIdx));
    // Check if this is the first time we are seeing this match.
    // If it is not, then mark it where it was first seen.
    if (curr_frame.is_initial_[match.trainIdx]) {
      curr_frame.is_initial_[match.trainIdx] = false;
      curr_frame.initial_ids_[match.trainIdx] =
          (past_frame.is_initial_[match.queryIdx])?
           past_frame.frame_ID_ :
           past_frame.initial_ids_[match.queryIdx];
    }
  }
  *correspondence = VisionFactor(
      past_frame.frame_ID_, curr_frame.frame_ID_, pairs);
}

void Frontend::AddOdometryFactor() {
  const Vector3f translation =
      prev_odom_rotation_.inverse() * (odom_translation_ -
      prev_odom_translation_);
  const Quaternionf rotation(odom_rotation_ * prev_odom_rotation_.inverse());
  odometry_factors_.push_back(OdometryFactor(
      curr_frame_ID_ - 1,
      curr_frame_ID_,
      translation,
      rotation));
}

void Frontend::UndistortFeaturePoints(vector<VisionFeature>* features_ptr) {
  vector<VisionFeature>& features = *features_ptr;
  const Vector2f p0(config_.intrinsics_left.cx, config_.intrinsics_left.cy);
  vector<cv::Point2f> distorted_pts;
  for (size_t i = 0; i < features.size(); ++i) {
    distorted_pts.push_back(EigenToOpenCV(features[i].pixel));
  }
  vector<cv::Point2f> undistorted_pts;
  cv::undistortPoints(distorted_pts,
                      undistorted_pts,
                      config_.camera_matrix_left,
                      config_.distortion_coeffs_left,
                      cv::noArray(),
                      config_.camera_matrix_left);
  CHECK_EQ(distorted_pts.size(), undistorted_pts.size());
  for (size_t i = 0; i < features.size(); ++i) {
    if (false) {
      printf("[%.3f %.3f] -> [%.3f %.3f]\n",
             distorted_pts[i].x,
             distorted_pts[i].y,
             undistorted_pts[i].x,
             undistorted_pts[i].y);
    }
    features[i].pixel = OpenCVToEigen(undistorted_pts[i]);
  }
}

bool Frontend::ObserveImage(const cv::Mat& left_image,
                            const cv::Mat& right_image,
                            double time) {
  // Check from the odometry if its time to run SLAM
  if (!OdomCheck()) {
    return false;
  }
  LOG(INFO) << "Observing Frame at " << frame_list_.size() << std::endl;
  Frame curr_frame, right_temp_frame;
  ExtractFeatures(left_image, &curr_frame);
  ExtractFeatures(right_image, &right_temp_frame);
  // Keep track of the points that are original to this frame.
  for (Frame& past_frame : frame_list_) {
    VisionFactor matches;
    GetFeatureMatches(&past_frame, &curr_frame, &matches);
    // TODO(Jack): verify that this conditional check does not mess up
    // book-keping.
    // if (matches.feature_matches.size() > config_.min_vision_matches) {
    //   vision_factors_.push_back(matches);
    // }
    vision_factors_.push_back(matches);
  }
  // Calculate the real 3D point locations.
  std::vector<Eigen::Vector3f> locations;
  Calculate3DLocations(&curr_frame,
                       &right_temp_frame,
                       &locations);
  vector<VisionFeature> features;
  for (uint64_t i = 0; i < curr_frame.keypoints_.size(); i++) {
    features.push_back(VisionFeature(
        i, OpenCVToEigen(curr_frame.keypoints_[i].pt), locations[i]));
  }
  UndistortFeaturePoints(&features);
  const Vector3f loc =  init_odom_rotation_.inverse() *
      (odom_translation_ - init_odom_translation_);
  const Quaternionf angle = odom_rotation_ * init_odom_rotation_.inverse();
  nodes_.push_back(SLAMNode(curr_frame_ID_,
                            odom_timestamp_,
                            RobotPose(loc,
                                      angle),
                            features));
  if (curr_frame_ID_ > 0) {
    AddOdometryFactor();
  }
  prev_odom_rotation_ = odom_rotation_;
  prev_odom_translation_ = odom_translation_;
  curr_frame_ID_++;
  if (config_.debug_images_ &&
      !frame_list_.empty()  &&
      !vision_factors_.empty()) {
    const slam_types::VisionFactor factor = vision_factors_.back();
    const Frame initial_frame = frame_list_.back();
    assert(factor.pose_initial == initial_frame.frame_ID_);
    debug_images_.push_back(
        CreateDebugImage(left_image, initial_frame, curr_frame, factor));
  }
  if (frame_list_.size() >= config_.frame_life_) {
    frame_list_.erase(frame_list_.begin());
  }
  frame_list_.push_back(curr_frame);
  return true;
}

vector<cv::Mat> Frontend::getDebugImages() {
  return debug_images_;
}

cv::Mat Frontend::GetLastDebugImage() {
  if (debug_images_.size() == 0) {
    return cv::Mat();
  }
  return debug_images_.back();
}


void Frontend::GetSLAMProblem(SLAMProblem* problem) const {
  *problem = slam_types::SLAMProblem(
      nodes_,
      vision_factors_,
      odometry_factors_);
}

/* --- Frame Implementation Code --- */

Frame::Frame(const vector<cv::KeyPoint>& keypoints,
             const cv::Mat& descriptors,
             const FrontendConfig& config,
             uint64_t frame_ID) {
  keypoints_ = keypoints;
  descriptors_ = descriptors;
  config_ = config;
  frame_ID_ = frame_ID;
  matcher_ = cv::BFMatcher::create(config_.bf_matcher_param_);
  is_initial_ = std::vector<bool>(keypoints_.size(), true);
  initial_ids_ = std::vector<int64_t>(keypoints_.size(), -1);
}

vector<cv::DMatch> Frame::GetMatches(const Frame& frame,
                                     double nn_match_ratio) {
  vector<vector<cv::DMatch>> matches;
  matcher_->knnMatch(descriptors_, frame.descriptors_, matches, 2);
  vector<cv::DMatch> best_matches;
  for (size_t i = 0; i < matches.size(); i++) {
    cv::DMatch first = matches[i][0];
    float dist1 = matches[i][0].distance;
    float dist2 = matches[i][1].distance;
    if (dist1 < config_.nn_match_ratio_ * dist2) {
      best_matches.push_back(first);
    }
  }
  return best_matches;
}

/* --- Config Implementation Code --- */

Matrix3f CameraMatrix(const CameraIntrinsics& I) {
  Matrix3f M;
  M << I.fx, 0, I.cx,
       0, I.fy, I.cy,
       0, 0, 1;
  return M;
}

void EigenToOpenCV(const Matrix<float, 3, 4>& m1,
                   cv::Mat* m2_ptr) {
  cv::Mat& m2 = *m2_ptr;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      m2.ptr<float>(r)[c] = m1(r, c);
    }
  }
}

FrontendConfig::FrontendConfig() {
  // Load Default values
  debug_images_ = FLAGS_v > 0;
  descriptor_extract_type_ = FrontendConfig::DescriptorExtractorType::AKAZE;
  best_percent_ = 0.3f;
  nn_match_ratio_ = 0.8f;
  frame_life_ = 10;
  min_odom_rotation = 10.0 / 180.0 * M_PI;
  min_odom_translation = 0.2;
  min_vision_matches = 10;

  // Taken from Campus-Jackal repo:
  // https://github.com/umass-amrl/Campus-Jackal/blob/master/hardware/
  //    calibration/PointGreyCalibration/6_Aug_24_18/jpp/
  //    pointgrey_calib_6_Aug_24_18.yaml
  intrinsics_left.fx = 527.873518;
  intrinsics_left.cx = 482.823413;
  intrinsics_left.fy = 527.276819;
  intrinsics_left.cy = 298.033945;
  intrinsics_left.k1 = -0.153137;
  intrinsics_left.k2 = 0.075666;
  intrinsics_left.p1 = -0.000227;
  intrinsics_left.p2 = -0.000320;
  intrinsics_left.k3 = -0.000227; //TODO(Joydeep): Should this be 0?

  intrinsics_right.fx = 530.158021;
  intrinsics_right.cx = 475.540633;
  intrinsics_right.fy = 529.682234;
  intrinsics_right.cy = 299.995465;
  intrinsics_right.k1 = -0.156833;
  intrinsics_right.k2 = 0.081841;
  intrinsics_right.p1 = -0.000779;
  intrinsics_right.p2 = -0.000356;
  intrinsics_right.k3 = -0.000779;

  camera_matrix_left = (cv::Mat_<float>(3, 3) <<
      intrinsics_left.fx, 0, intrinsics_left.cx,
      0, intrinsics_left.fy, intrinsics_left.cy,
      0, 0, 1);

  camera_matrix_right = (cv::Mat_<float>(3, 3) <<
      intrinsics_right.fx, 0, intrinsics_right.cx,
      0, intrinsics_right.fy, intrinsics_right.cy,
      0, 0, 1);

  const Matrix3f K_left = CameraMatrix(intrinsics_left);
  Matrix<float, 3, 4> A_left;
  A_left << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0;
  const Matrix<float, 3, 4> P_left = K_left * A_left;

  const Matrix3f K_right = CameraMatrix(intrinsics_right);
  Matrix<float, 3, 4> A_right;
  A_right <<
      0.999593617649873, 0.021411909431148, -0.018818333830411,
          -0.131707087331978,
      -0.021140534893290, 0.999671312094879, 0.014503294761121,
          0.003232397463343,
      0.019122691705565, -0.014099571235136, 0.999717722536176,
          -0.001146108483477;
  const Matrix<float, 3, 4> P_right = K_right * A_right;

  projection_left = cv::Mat_<float>(3, 4);
  projection_right = cv::Mat_<float>(3, 4);

  EigenToOpenCV(P_left, &projection_left);
  EigenToOpenCV(P_right, &projection_right);

  distortion_coeffs_left = (cv::Mat_<float>(5, 1) <<
      intrinsics_left.k1,
      intrinsics_left.k2,
      intrinsics_left.p1,
      intrinsics_left.p2,
      intrinsics_left.k3);
  if (false) {
    std::cout << "\n\nprojection_left:\n";
    std::cout << projection_left << "\n";
    std::cout << "\n\nprojection_right:\n";
    std::cout << projection_right<< "\n";
    exit(0);
  }
  // TODO: Later we should change how config works because this is done every
  // time a frame is created.
}

}  // namespace slam
