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
#include <ros/ros.h>

#include "slam_frame.h"
#include "slam_frontend_backend_comm.h"

namespace slam {
class Frontend {
 public:
  Frontend(ros::NodeHandle& n, const std::string& config_path);
  // Observe a new image. Extract features, and match to past frames.
  void ObserveImage(const cv::Mat& image, 
		    double time,
		    uint64_t frame_ID);

  void ObserveOdometry(const Eigen::Vector3f& translation,
                       const Eigen::Quaternionf& rotation,
                       double time);
  
 private:
  slam::FrontendBackendComm* broadcaster_;
  FrontendConfig config_;
  cv::NormTypes bf_matcher_param_;
  std::vector<Frame> frame_list_;
  cv::Ptr<cv::Feature2D> descriptor_extractor_;
  cv::Ptr<cv::FastFeatureDetector> fast_feature_detector_;
  uint64_t frame_ID_count_ = 0;
};
}  // namespace slam

#endif   // __SLAM_FRONTEND_H__
