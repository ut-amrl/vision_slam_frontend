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
  // Camera pixel location of feature.
  Eigen::Vector2f pixel;
  // Calculated 3d location
  Eigen::Vector3f location;
  // Default constructor: do nothing.
  VisionFeature() {}
  // Convenience constructor: initialize everything.
  VisionFeature(uint64_t id,
                const Eigen::Vector2f& p,
                const Eigen::Vector3f& loc) :
      id(id), pixel(p), location(loc) {}
};

struct FeatureMatch {
  // Feature ID from the initial pose.
  uint64_t id_initial;
  // Feature ID from current pose.
  uint64_t id_current;
  // Default constructor: do nothing.
  FeatureMatch() {}
  // Convenience constructor: initialize everything.
  FeatureMatch(uint64_t id_initial,
               uint64_t id_current) :
    id_initial(id_initial),
    id_current(id_current) {}
};

struct VisionFactor {
  // ID of the pose where the features were *first* observed.
  uint64_t pose_initial;
  // ID of second pose.
  uint64_t pose_current;
  // Pair of feature ID from first pose, and feature ID from second pose,
  // and feature ID from initial pose.
  std::vector<FeatureMatch> feature_matches;
  // Default constructor: do nothing.
  VisionFactor() {}
  // Convenience constructor: initialize everything.
  VisionFactor(
      uint64_t pose_initial,
      uint64_t pose_current,
      const std::vector<slam_types::FeatureMatch>& feature_matches) :
      pose_initial(pose_initial), pose_current(pose_current),
      feature_matches(feature_matches) {}
};

struct RobotPose {
  // Robot location.
  Eigen::Vector3f loc;
  // Robot angle: rotates points from robot frame to global.
  Eigen::Quaternionf angle;
  // Default constructor: do nothing.
  RobotPose() {}
  // Convenience constructor: initialize everything.
  RobotPose(const Eigen::Vector3f& loc,
            const Eigen::Quaternionf& angle) :
      loc(loc), angle(angle) {}
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
struct OdometryFactor {
  // ID of first pose.
  uint64_t pose_i;
  // ID of second pose.
  uint64_t pose_j;
  // Translation to go from pose i to pose j.
  Eigen::Vector3f translation;
  // Rotation to go from pose i to pose j.
  Eigen::Quaternionf rotation;
  // Default constructor: do nothing.
  OdometryFactor() {}
  // Convenience constructor: initialize everything.
  OdometryFactor(uint64_t pose_i,
                         uint64_t pose_j,
                         Eigen::Vector3f translation,
                         Eigen::Quaternionf rotation) :
      pose_i(pose_i), pose_j(pose_j), translation(translation),
      rotation(rotation) {}
};

struct SLAMNode {
  // Pose ID.
  uint64_t id;
  // Timestamp.
  double timestamp;
  // 6DOF parameters.
  RobotPose pose;
  // Observed vision features.
  std::vector<VisionFeature> features;
  // Default constructor: do nothing.
  SLAMNode() {}
  // Convenience constructor, initialize all components.
  SLAMNode(uint64_t id,
           double timestamp,
           const RobotPose& pose,
           const std::vector<VisionFeature>& features) :
      id(id), timestamp(timestamp), pose(pose), features(features) {}
};

struct SLAMProblem {
  // Nodes in the pose graph.
  std::vector<SLAMNode> nodes;
  // Vision correspondences.
  std::vector<VisionFactor> vision_factors;
  // Odometry / IMU correspondences.
  std::vector<OdometryFactor> odometry_factors;
  // Default constructor, do nothing.
  SLAMProblem() {}
  // Convenience constructor for initialization.
  SLAMProblem(const std::vector<SLAMNode>& nodes,
              const std::vector<VisionFactor>& vision_factors,
              const std::vector<OdometryFactor>& odometry_factors) :
      nodes(nodes),
      vision_factors(vision_factors),
      odometry_factors(odometry_factors) {}
};

}  // namespace slam_types

#endif  // __SLAM_TYPES_H__
