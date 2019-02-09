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

#include "slam_config.h"

slam::FrontendConfig::FrontendConfig() {
  //Load Default values
  descriptor_extract_type_ = FrontendConfig::DescriptorExtractorType::FREAK;
  best_percent_ = 0.3f;
  nn_match_ratio_ = 0.8f;
  frame_life_ = 5;
  bf_matcher_param_ = cv::NORM_HAMMING;
}
