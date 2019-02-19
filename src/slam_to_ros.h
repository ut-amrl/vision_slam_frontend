#ifndef __SLAM_TO_ROS_H__
#define __SLAM_TO_ROS_H__

#include "vision_slam_frontend/RobotPose.h"
#include "vision_slam_frontend/SLAMNode.h"
#include "vision_slam_frontend/VisionCorrespondence.h"
#include "vision_slam_frontend/VisionCorrespondencePair.h"
#include "vision_slam_frontend/VisionFeature.h"
#include "slam_types.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

static vision_slam_frontend::VisionCorrespondencePair
VisionCorrespondencePairToRos(const slam_types::FeatureMatch& pair) {
  vision_slam_frontend::VisionCorrespondencePair ros_pair;
  ros_pair.pose_i_index = pair.id_i;
  ros_pair.pose_j_index = pair.id_j;
  ros_pair.pose_initial = pair.pose_initial;
  ros_pair.pose_initial_index = pair.id_initial;
  return ros_pair;
}

static vision_slam_frontend::VisionFeature
VisionFeatureToRos(const slam_types::VisionFeature& feature) {
  vision_slam_frontend::VisionFeature ros_feature;
  ros_feature.descriptor_id = feature.descriptor_id;
  ros_feature.id = feature.id;
  ros_feature.pixel.x = feature.pixel.x();
  ros_feature.pixel.y = feature.pixel.y();
  ros_feature.pixel.z = 0;
  //Add Feature Point when used.
  return ros_feature;
}

static vision_slam_frontend::RobotPose
RobotPoseToRos(const slam_types::RobotPose& rp) {
  vision_slam_frontend::RobotPose ros_rp;
  ros_rp.loc.x = rp.loc.x();
  ros_rp.loc.y = rp.loc.y();
  ros_rp.loc.z = rp.loc.z();
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
  ros_node.pose = RobotPoseToRos(node.pose);
  for (auto p: node.features) {
    ros_node.features.push_back(VisionFeatureToRos(p));
  }
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
