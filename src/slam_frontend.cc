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
#include "opencv2/core/eigen.hpp"

#include "slam_frontend.h"

using slam_types::RobotPose;
using slam_types::SLAMNode;
using slam_types::SLAMProblem;
using slam_types::OdometryFactor;
using slam_types::VisionFactor;
using slam_types::FeatureMatch;
using slam_types::VisionFeature;
using std::cout;
using std::endl;
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

/* Debug Image Generators */
cv::Mat CreateStereoDebugImage(const Frame& frame_left,
                               const Frame& frame_right,
                               const VisionFactor& matches) {
  int left_cols = frame_left.debug_image_.cols;
  cv::Mat return_image;
  cv::hconcat(frame_left.debug_image_, frame_right.debug_image_, return_image);
  // Convert debug image to RGB so we can draw other colors on it.
  cv::cvtColor(return_image, return_image, cv::COLOR_GRAY2BGR);
  // Draw all the stereo matches on the final image.
  for (FeatureMatch match : matches.feature_matches) {
    cv::Point2f left_point =
        frame_left.keypoints_[match.feature_idx_current].pt;
    cv::circle(return_image, left_point, 5, CV_RGB(255, 0, 0));
    cv::Point2f right_point =
        frame_right.keypoints_[match.feature_idx_initial].pt;
    right_point.x += left_cols;
    cv::circle(return_image, right_point, 5, CV_RGB(255, 0, 0));
    cv::line(return_image,
             left_point,
             right_point,
             cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
  }
  return return_image;
}

cv::Mat CreateMatchDebugImage(const Frame& frame_initial,
                              const Frame& frame_current,
                              const VisionFactor& corr) {
  cv::Mat return_image = frame_current.debug_image_.clone();
  cv::cvtColor(return_image, return_image, cv::COLOR_GRAY2RGB);
  for (const slam_types::FeatureMatch c : corr.feature_matches) {
    cv::circle(return_image,
               frame_initial.keypoints_[c.feature_idx_initial].pt,
               5, CV_RGB(255, 0, 0));
    cv::line(return_image,
             frame_initial.keypoints_[c.feature_idx_initial].pt,
             frame_current.keypoints_[c.feature_idx_current].pt,
             CV_RGB(0, 255, 0));
  }
  return return_image;
}

void Frontend::Calculate3DPoints(Frame* left_frame,
                                 Frame* right_frame,
                                 vector<Vector3f>* points) {
  // Convert keypoints into array of points.
  std::vector<cv::Point2f> left_points;
  std::vector<cv::Point2f> right_points;
  // The reason we pass in right frame first is because we don't want to modify
  // the left frames is_initial_ book-keeping.
  // This means right will be the 'initial' and the left_frame
  // will be the 'current' frame.
  VisionFactor* matches;
  // Assure that every point has a match.
  float best_percent = config_.best_percent_;
  config_.best_percent_ = 1.0;
  matches = GetFeatureMatches(right_frame, left_frame);
  config_.best_percent_ = best_percent;
  if (matches->feature_matches.size() == 0) {
    return;
  }
  for (FeatureMatch match : matches->feature_matches) {
    const auto& left_pt = left_frame->keypoints_[match.feature_idx_current].pt;
    const auto& right_pt =
        right_frame->keypoints_[match.feature_idx_initial].pt;
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
  cv::triangulatePoints(config_.projection_left,
                        config_.projection_right,
                        left_points,
                        right_points,
                        triangulated_points);
  // Make sure all keypoints are matched to something.
  CHECK_EQ(triangulated_points.cols, matches->feature_matches.size());
  for (int64_t c = 0; c < triangulated_points.cols; ++c) {
    cv::Mat col = triangulated_points.col(c);
    points->push_back(
      Vector3f(col.at<float>(0, 0),
               col.at<float>(1, 0),
               col.at<float>(2, 0)) /col.at<float>(3, 0));
  }
  // Generate Debug Images if needed
  if (config_.debug_images_) {
    debug_stereo_images_.push_back(CreateStereoDebugImage(*left_frame,
                                                          *right_frame,
                                                          *matches));
  }
  free(matches);
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
  matcher_ = cv::BFMatcher::create(config_.bf_matcher_param_);
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
  *frame = Frame(frame_keypoints, frame_descriptors, curr_frame_ID_);
}

VisionFactor* Frontend::GetFeatureMatches(Frame* past_frame_ptr,
                                          Frame* curr_frame_ptr) {
  Frame& past_frame = *past_frame_ptr;
  Frame& curr_frame = *curr_frame_ptr;
  vector<FeatureMatch> pairs;
  vector<cv::DMatch> matches =
        GetMatches(past_frame, curr_frame, config_.nn_match_ratio_);
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
//   printf("%lu\n", pairs.size());
  return new VisionFactor(past_frame.frame_ID_, curr_frame.frame_ID_, pairs);
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
  if (features.size() == 0) {
    return;
  }
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

static float stereo_ambig_constraint = 10000;
void Frontend::RemoveAmbigStereo(Frame* left,
                                 Frame* right,
                                 const std::vector<cv::DMatch> stereo_matches) {
  // Get the left and right points.
  std::vector<cv::Point2f> left_points;
  std::vector<cv::Point2f> right_points;
  for (uint64 m = 0; m < stereo_matches.size(); m++) {
    cv::DMatch match = stereo_matches[m];
    left_points.push_back(left->keypoints_[match.queryIdx].pt);
    right_points.push_back(right->keypoints_[match.trainIdx].pt);
  }
//   cv::Mat fund = cv::findFundamentalMat(left_points, right_points);
  // Convert to Eigen format
  std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
  cv::Mat left_descs, right_descs;
  float avg_constraint = 0.0f;
  for (uint64 m = 0; m < stereo_matches.size(); m++) {
    // Transform into homogenous coordinates
    Eigen::Matrix<float, 3, 1> left_ph;
    left_ph[0] = left_points[m].x;
    left_ph[1] = left_points[m].y;
    left_ph[2] = 1.0f;
    Eigen::Matrix<float, 3, 1> right_ph;
    right_ph[0] = right_points[m].x;
    right_ph[1] = right_points[m].y;
    right_ph[2] = 1.0f;
    float constraint =
        (left_ph.transpose() * config_.fundamental *right_ph).norm();
    avg_constraint += constraint;
    if (constraint <= stereo_ambig_constraint) {
      // This set of points is considered non-ambigious, keep it.
      cv::DMatch match = stereo_matches[m];
      left_keypoints.push_back(left->keypoints_[match.queryIdx]);
      right_keypoints.push_back(right->keypoints_[match.trainIdx]);
      left_descs.push_back(left->descriptors_.row(match.queryIdx));
      right_descs.push_back(right->descriptors_.row(match.trainIdx));
    }
  }
  float padding_from_avg = 1000;
  stereo_ambig_constraint =
      avg_constraint / stereo_matches.size() + padding_from_avg;
  printf("Avg Constraint: %f\n", avg_constraint / stereo_matches.size());
  // Set the left and right data to the update non-ambigious data.
  *left = Frame(left_keypoints, left_descs, left->frame_ID_);
  *right = Frame(right_keypoints, right_descs, right->frame_ID_);
}

bool Frontend::ObserveImage(const cv::Mat& left_image,
                            const cv::Mat& right_image,
                            double time) {
  // Check from the odometry if its time to run SLAM
  if (!OdomCheck()) {
    return false;
  }
  if (FLAGS_v > 2) {
    printf("Observing Frame at %d\n", static_cast<int>(frame_list_.size()));
  }
  Frame curr_frame, right_temp_frame;
  ExtractFeatures(left_image, &curr_frame);
  ExtractFeatures(right_image, &right_temp_frame);
  // Remove ambigious stereo matching points from left's and right's points.
  std::vector<cv::DMatch> stereo_matches = GetMatches(curr_frame,
                                                      right_temp_frame,
                                                      config_.nn_match_ratio_);
  RemoveAmbigStereo(&curr_frame, &right_temp_frame, stereo_matches);
  // If we are running a debug version, attach image to frame.
  if (config_.debug_images_) {
    curr_frame.debug_image_ = left_image;
    right_temp_frame.debug_image_ = right_image;
  }
  // Keep track of the points that are original to this frame.
  for (Frame& past_frame : frame_list_) {
    VisionFactor* matches;
    matches = GetFeatureMatches(&past_frame, &curr_frame);
    // TODO(Jack): verify that this conditional check does not mess up
    // book-keping.
    // if (matches.feature_matches.size() > config_.min_vision_matches) {
    //   vision_factors_.push_back(matches);
    // }
    vision_factors_.push_back(*matches);
    free(matches);
  }
  // Calculate the depths of the points.
  vector<Vector3f> points;
  Calculate3DPoints(&curr_frame, &right_temp_frame, &points);
  vector<VisionFeature> features;
  for (uint64_t i = 0; i < curr_frame.keypoints_.size(); i++) {
    features.push_back(VisionFeature(
        i, OpenCVToEigen(curr_frame.keypoints_[i].pt), points[i]));
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
    CHECK_EQ(factor.pose_idx_initial, initial_frame.frame_ID_);
    debug_images_.push_back(
        CreateMatchDebugImage(initial_frame, curr_frame, factor));
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

vector<cv::Mat> Frontend::getDebugStereoImages() {
  return debug_stereo_images_;
}


cv::Mat Frontend::GetLastDebugImage() {
  if (debug_images_.size() == 0) {
    return cv::Mat();
  }
  return debug_images_.back();
}

cv::Mat Frontend::GetLastDebugStereoImage() {
  if (debug_stereo_images_.size() == 0) {
    return cv::Mat();
  }
  return debug_stereo_images_.back();
}


void Frontend::GetSLAMProblem(SLAMProblem* problem) const {
  *problem = slam_types::SLAMProblem(
      nodes_,
      vision_factors_,
      odometry_factors_);
}

int Frontend::GetNumPoses() {
  return nodes_.size();
}

/* --- Frame Implementation Code --- */

Frame::Frame(const vector<cv::KeyPoint>& keypoints,
             const cv::Mat& descriptors,
             uint64_t frame_ID) {
  keypoints_ = keypoints;
  descriptors_ = descriptors;
  frame_ID_ = frame_ID;
  is_initial_ = std::vector<bool>(keypoints_.size(), true);
  initial_ids_ = std::vector<int64_t>(keypoints_.size(), -1);
}

vector<cv::DMatch> Frontend::GetMatches(const Frame& frame_query,
                                        const Frame& frame_train,
                                        double nn_match_ratio) {
  vector<vector<cv::DMatch>> matches;
  matcher_->knnMatch(frame_query.descriptors_,
                     frame_train.descriptors_,
                     matches, 2);
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

FrontendConfig::FrontendConfig() {
  // Load Default values
  debug_images_ = true;
  descriptor_extract_type_ = FrontendConfig::DescriptorExtractorType::AKAZE;
  best_percent_ = 0.3f;
  nn_match_ratio_ = 0.6f;
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
  intrinsics_left.k3 = 0;

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

  const Vector3f XT(-0.01, 0.06, 0.5299999713897705);
  Matrix3f RT;
  RT << 0.009916590468,  -0.2835522866,   0.9589055021,
        -0.9998698619, -0.01501486552, 0.005900269087,
        0.01272480238,  -0.9588392225,  -0.2836642819;
  left_cam_to_robot = Eigen::Translation3f(XT) * RT;
  projection_left = cv::Mat(3, 4, CV_32F);
  projection_right = cv::Mat(3, 4, CV_32F);
  cv::eigen2cv(P_left, projection_left);
  cv::eigen2cv(P_right, projection_right);
  distortion_coeffs_left = (cv::Mat_<float>(5, 1) <<
      intrinsics_left.k1,
      intrinsics_left.k2,
      intrinsics_left.p1,
      intrinsics_left.p2,
      intrinsics_left.k3);
  distortion_coeffs_right = (cv::Mat_<float>(5, 1) <<
    intrinsics_right.k1,
    intrinsics_right.k2,
    intrinsics_right.p1,
    intrinsics_right.p2,
    intrinsics_right.k3);
  Matrix3f ecam_left, ecam_right;
  cv::cv2eigen(camera_matrix_left, ecam_left);
  cv::cv2eigen(camera_matrix_right, ecam_right);
  Matrix<float, 3, 1> A = ecam_left * RT.transpose() * XT;
  Matrix3f C;
  C << 0.0, -A[2], A[1], A[2], 0.0, -A[1], -A[2], A[1], 0.0;
  fundamental =
      ecam_right.inverse().transpose() * RT * ecam_left.transpose() * C;
  if (false) {
    std::cout << "\n\nprojection_left:\n";
    std::cout << projection_left << "\n";
    std::cout << "\n\nprojection_right:\n";
    std::cout << projection_right<< "\n";
    exit(0);
  }
}

}  // namespace slam
