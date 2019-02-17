// Copyright (c) 2018 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __SLAM_TYPES_H__
#define __SLAM_TYPES_H__

#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace slam_types {

struct VisionFeature {
  // ID of feature, unique to node.
  uint64_t id;
  // Descriptor ID of feature. In practice this will be dependent on the
  // descriptor type.
  uint64_t descriptor_id;
  // Camera pixel location of feature.
  Eigen::Vector2f pixel;
  // Default constructor: do nothing.
  VisionFeature() {}
  // Convenience constructor: initialize everything.
  VisionFeature(uint64_t id,
                uint64_t descriptor_id,
                const Eigen::Vector2f& p) :
      id(id), descriptor_id(descriptor_id), pixel(p) {}
};

struct VisionCorrespondencePair {
  //ID of initial pose.
  uint64_t pose_initial;
  //feature ID from initial pose.
  uint64_t pose_initial_id;
  //feature ID from first pose.
  uint64_t pose_i_id;
  //feature ID from second pose.
  uint64_t pose_j_id;
  // Default constructor: do nothing.
  VisionCorrespondencePair() {}
  // Convenience constructor: initialize everything.
  VisionCorrespondencePair(uint64_t pose_i_idx,
                           uint64_t pose_j_idx,
                           uint64_t pose_initial,
                           uint64_t pose_initial_idx) :
      pose_initial(pose_initial),
      pose_initial_id(pose_initial_idx),
      pose_i_id(pose_i_idx),
      pose_j_id(pose_j_idx) {}
};

struct VisionCorrespondence {
  // ID of first pose.
  uint64_t pose_i;
  // ID of second pose.
  uint64_t pose_j;
  // Pair of feature ID from first pose, and feature ID from second pose,
  // and feature ID from initial pose.
  std::vector<VisionCorrespondencePair> feature_matches;
  // Default constructor: do nothing.
  VisionCorrespondence() {}
  // Convenience constructor: initialize everything.
  VisionCorrespondence(
      uint64_t pose_i,
      uint64_t pose_j,
      const std::vector<slam_types::VisionCorrespondencePair>& pairs) :
      pose_i(pose_i), pose_j(pose_j), feature_matches(pairs) {}
};

struct RobotPose {
  // Timestamp.
  double timestamp;
  // Robot location.
  Eigen::Vector3f loc;
  // Robot angle: rotates points from robot frame to global.
  Eigen::Quaternionf angle;
  // Default constructor: do nothing.
  RobotPose() {}
  // Convenience constructor: initialize everything.
  RobotPose(double t,
            const Eigen::Vector3f& loc,
            const Eigen::Quaternionf& angle) :
      timestamp(t), loc(loc), angle(angle) {}
  // Return a transform from the robot to the world frame for this pose.
  Eigen::Affine3f RobotToWorldTf() const {
    return (Eigen::Translation3f(loc) * angle);
  }
  // Return a transform from the world to the robot frame for this pose.
  Eigen::Affine3f WorldToRobotTf() const {
    return ((Eigen::Translation3f(loc) * angle).inverse());
  }
};

// Represents an IMU or odometry, or fused IMU-odometry observation.
struct OdometryCorrespondence {
  // ID of first pose.
  uint64_t pose_i;
  // ID of second pose.
  uint64_t pose_j;
  // Translation to go from pose i to pose j.
  Eigen::Vector3f translation;
  // Rotation to go from pose i to pose j.
  Eigen::Quaternionf rotation;
};

struct SLAMNode {
  // Pose ID.
  uint64_t id;
  // 6DOF parameters.
  RobotPose pose;
  // Observed vision features.
  std::vector<VisionFeature> features;
  // Default constructor: do nothing.
  SLAMNode() {}
  // Convenience constructor, initialize all components.
  SLAMNode(uint64_t id,
           const RobotPose& pose,
           const std::vector<VisionFeature>& features) :
      id(id), pose(pose), features(features) {}
};

struct SLAMProblem {
  // Nodes in the pose graph.
  std::vector<SLAMNode> nodes;

  // Vision correspondences.
  std::vector<VisionCorrespondence> vision;

  // Odometry / IMU correspondences.
  std::vector<OdometryCorrespondence> odometry;
};

}  // namespace vio_types

#endif // __SLAM_TYPES_H__
