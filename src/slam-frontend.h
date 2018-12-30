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

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "opencv2/opencv.hpp"

#ifndef __SLAM_FRONTEND_H__
#define __SLAM_FRONTEND_H__

namespace slam {
class Frontend {
 public:
  // Observe a new image. Extract features, and match to past frames.
  void ObserveImage(const cv::Mat& image, double time);

  void ObserveOdometry(const Eigen::Vector3f& translation,
                       const Eigen::Quaternionf& rotation,
                       double time);

 private:

};
}  // namespace slam

#endif   // __SLAM_FRONTEND_H__
