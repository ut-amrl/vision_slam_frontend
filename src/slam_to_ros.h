#ifndef __SLAM_TO_ROS_H__
#define __SLAM_TO_ROS_H__

#include "vision_slam_frontend/RobotPose.h"
#include "vision_slam_frontend/SLAMNode.h"
#include "vision_slam_frontend/VisionCorrespondence.h"
#include "vision_slam_frontend/VisionCorrespondencePair.h"
#include "vision_slam_frontend/VisionFeature.h"
#include "slam_types.h"

static vision_slam_frontend::VisionCorrespondencePair
VisionCorrespondencePairToRos(const slam_types::VisionCorrespondencePair& pair) {
  vision_slam_frontend::VisionCorrespondencePair ros_pair;
  ros_pair.pose_i_index = pair.pose_i_id;
  ros_pair.pose_j_index = pair.pose_j_id;
  ros_pair.pose_initial = pair.pose_initial;
  ros_pair.pose_initial_index = pair.pose_initial_id;
  return ros_pair;
}

static vision_slam_frontend::VisionFeature
VisionFeatureToRos(const slam_types::VisionFeature& feature) {
  vision_slam_frontend::VisionFeature ros_feature;
  ros_feature.descriptor_id = feature.descriptor_id;
  ros_feature.id = feature.id;
  ros_feature.pixel.x = feature.pixel[0];
  ros_feature.pixel.y = feature.pixel[1];
  ros_feature.pixel.z = 0;
  //Add Feature Point when used.
  return ros_feature;
}

static vision_slam_frontend::SLAMNode
SLAMNodeToRos(const slam_types::SLAMNode& node) {
  vision_slam_frontend::SLAMNode ros_node;
  ros_node.id = node.id;
  // Add robot pose when used.
  // ros_node.pose = node.pose;
  for (auto p: node.features) {
    ros_node.features.push_back(VisionFeatureToRos(p));
  }
  ros_node.is_vision_node = node.is_vision_node;
  return ros_node;
}

static vision_slam_frontend::VisionCorrespondence
VisionCorrespondenceToRos(const slam_types::VisionCorrespondence& corr) {
  vision_slam_frontend::VisionCorrespondence ros_corr;
  ros_corr.pose_i = corr.pose_i;
  ros_corr.pose_j = corr.pose_j;
  for (auto m: corr.feature_matches) {
    ros_corr.feature_matches.push_back(VisionCorrespondencePairToRos(m));
  }
  return ros_corr;
}

#endif //__SLAM_TO_ROS_H__