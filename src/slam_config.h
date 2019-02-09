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
\brief   A vision SLAM Config
\author  John Bachman, (C) 2019
*/
//========================================================================

#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

#include <opencv2/opencv.hpp>

namespace slam {
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
  void setBFMatcherParam(cv::NormTypes bf_matcher_param) { bf_matcher_param_ = bf_matcher_param; }
 private:
  DescriptorExtractorType  descriptor_extract_type_;
  double best_percent_;
  double nn_match_ratio_;
  uint32_t frame_life_;
  cv::NormTypes bf_matcher_param_;
}; 
};

#endif // SLAM_CONFIG_H
