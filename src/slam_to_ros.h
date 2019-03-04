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
\brief   Slam types to ros message conversions
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#ifndef __SLAM_TO_ROS_H__
#define __SLAM_TO_ROS_H__

#include "vision_slam_frontend/RobotPose.h"
#include "vision_slam_frontend/SLAMNode.h"
#include "vision_slam_frontend/FeatureMatch.h"
#include "vision_slam_frontend/VisionFeature.h"
#include "vision_slam_frontend/VisionFactor.h"
#include "vision_slam_frontend/SLAMProblem.h"
#include "vision_slam_frontend/OdometryFactor.h"
#include "slam_types.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

static vision_slam_frontend::FeatureMatch
FeatureMatchToRos(const slam_types::FeatureMatch& pair) {
  vision_slam_frontend::FeatureMatch ros_pair;
  ros_pair.id_current = pair.id_current;
  ros_pair.id_initial = pair.id_initial;
  return ros_pair;
}

static vision_slam_frontend::VisionFeature
VisionFeatureToRos(const slam_types::VisionFeature& feature) {
  vision_slam_frontend::VisionFeature ros_feature;
  ros_feature.pixel.x = feature.pixel.x();
  ros_feature.pixel.y = feature.pixel.y();
  ros_feature.pixel.z = 0;
  ros_feature.point3d.x = feature.point3d.x();
  ros_feature.point3d.y = feature.point3d.y();
  ros_feature.point3d.z = feature.point3d.z();
  ros_feature.id = feature.id;
//   CHECK(std::isfinite(ros_feature.point3d.x));
//   CHECK(std::isfinite(ros_feature.point3d.y));
//   CHECK(std::isfinite(ros_feature.point3d.z));
  return ros_feature;
}

static vision_slam_frontend::RobotPose
RobotPoseToRos(const slam_types::RobotPose& rp) {
  vision_slam_frontend::RobotPose ros_rp;
  ros_rp.loc.x = rp.loc[0];
  ros_rp.loc.y = rp.loc[1];
  ros_rp.loc.z = rp.loc[2];
  ros_rp.angle.w = rp.angle.w();
  ros_rp.angle.x = rp.angle.x();
  ros_rp.angle.y = rp.angle.y();
  ros_rp.angle.z = rp.angle.z();
  return ros_rp;
}

static vision_slam_frontend::SLAMNode
SLAMNodeToRos(const slam_types::SLAMNode& node) {
  vision_slam_frontend::SLAMNode ros_node;
  ros_node.id = node.id;
  ros_node.timestamp = node.timestamp;
  ros_node.pose = RobotPoseToRos(node.pose);
  for (auto p : node.features) {
    ros_node.features.push_back(VisionFeatureToRos(p));
  }
  return ros_node;
}

static vision_slam_frontend::VisionFactor
VisionFactorToRos(const slam_types::VisionFactor& corr) {
  vision_slam_frontend::VisionFactor ros_corr;
  ros_corr.pose_current = corr.pose_current;
  ros_corr.pose_initial = corr.pose_initial;
  for (auto m : corr.feature_matches) {
    ros_corr.feature_matches.push_back(FeatureMatchToRos(m));
  }
  return ros_corr;
}

static vision_slam_frontend::OdometryFactor
OdometryFactorToRos(const slam_types::OdometryFactor& odom_f) {
  vision_slam_frontend::OdometryFactor ros_odom_f;
  ros_odom_f.pose_i = odom_f.pose_i;
  ros_odom_f.pose_j = odom_f.pose_j;
  ros_odom_f.rotation.w = odom_f.rotation.w();
  ros_odom_f.rotation.x = odom_f.rotation.x();
  ros_odom_f.rotation.y = odom_f.rotation.y();
  ros_odom_f.rotation.z = odom_f.rotation.z();
  ros_odom_f.translation.x = odom_f.translation[0];
  ros_odom_f.translation.y = odom_f.translation[1];
  ros_odom_f.translation.z = odom_f.translation[2];
  return ros_odom_f;
}

static vision_slam_frontend::SLAMProblem
SLAMProblemToRos(const slam_types::SLAMProblem& problem) {
  vision_slam_frontend::SLAMProblem ros_problem;
  for (auto node : problem.nodes) {
    ros_problem.nodes.push_back(SLAMNodeToRos(node));
  }
  for (auto vfactor : problem.vision_factors) {
    ros_problem.vision_factors.push_back(VisionFactorToRos(vfactor));
  }
  for (auto ofactor : problem.odometry_factors) {
    ros_problem.odometry_factors.push_back(OdometryFactorToRos(ofactor));
  }
  return ros_problem;
}

#endif  // __SLAM_TO_ROS_H__
