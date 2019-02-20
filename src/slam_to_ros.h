#ifndef __SLAM_TO_ROS_H__
#define __SLAM_TO_ROS_H__

#include "vision_slam_frontend/RobotPose.h"
#include "vision_slam_frontend/SLAMNode.h"
#include "vision_slam_frontend/FeatureMatch.h"
#include "vision_slam_frontend/VisionFeature.h"
#include "vision_slam_frontend/VisionFactor.h"
#include "slam_types.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

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
  ros_feature.pixel.x = feature.pixel[0];
  ros_feature.pixel.y = feature.pixel[1];
  ros_feature.pixel.z = feature.pixel[2];
  ros_feature.id = feature.id;
  //Add Feature Point when used.
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
  for (auto p: node.features) {
    ros_node.features.push_back(VisionFeatureToRos(p));
  }
  return ros_node;
}

static vision_slam_frontend::VisionFactor
VisionFactorToRos(const slam_types::VisionFactor& corr) {
  vision_slam_frontend::VisionFactor ros_corr;
  ros_corr.pose_current = corr.pose_current;
  ros_corr.pose_initial = corr.pose_initial;
  for (auto m: corr.feature_matches) {
    ros_corr.feature_matches.push_back(FeatureMatchToRos(m));
  }
  return ros_corr;
}

#endif //__SLAM_TO_ROS_H__
