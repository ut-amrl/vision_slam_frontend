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

#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"


#ifndef __VIO_TYPES_H__
#define __VIO_TYPES_H__

using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Quaternionf;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Translation3f;
using Eigen::Vector2f;
using Eigen::Vector3f;

namespace vio_types {

struct VisionFeature {
  // ID of feature, unique to node.
  uint64_t id;
  // Descriptor ID of feature. In practice this will be dependent on the
  // descriptor type.
  uint64_t descriptor_id;
  // Camera pixel location of feature.
  Vector2f pixel;
  // Ground truth 3D location of feature points, in local coordinate frame.
  Vector3f point;
  // Default constructor: do nothing.
  VisionFeature() {}
  // Convenience constructor: initialize everything.
  VisionFeature(uint64_t id,
                uint64_t descriptor_id,
                const Vector2f& p,
                const Vector3f& ground_truth) :
      id(id), descriptor_id(descriptor_id), pixel(p), point(ground_truth) {}
};

struct VisionCorrespondence {
  // ID of initial pose, first ever sighting.
  uint64_t pose_initial;
  // Pair of feature ID from first pose, and feature ID from second pose.
  std::pair<uint64_t, uint64_t> feature_match;
  VisionCorrespondence (uint64_t first_pose, 
		        uint64_t match_idx_1, 
			uint64_t match_idx_2) : 
      pose_initial(first_pose) {
	feature_match = std::pair<uint64_t, uint64_t>(match_idx_1, match_idx_2);
      }
};

struct VisionCorrespondences {
  // ID of first pose.
  uint64_t pose_i;
  // ID of second pose.
  uint64_t pose_j;
  std::vector<VisionCorrespondence> correspondences;
  VisionCorrespondences(uint64_t frame_old, uint64_t frame_new) {
    pose_i = frame_old;
    pose_j = frame_new;
  }
  void AddMatch(VisionCorrespondence &v) {
    correspondences.push_back(v);
  }
};

struct RobotPose {
  // Robot location.
  Vector3f loc;
  // Robot angle: rotates points from robot frame to global.
  AngleAxisf angle;
  // Default constructor: do nothing.
  RobotPose() {}
  // Convenience constructor: initialize everything.
  RobotPose(const Vector3f& loc, const AngleAxisf& angle) :
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
struct OdometryCorrespondence {
  // ID of first pose.
  uint64_t pose_i;
  // ID of second pose.
  uint64_t pose_j;
  // Translation to go from pose i to pose j.
  Vector3f translation;
  // Rotation to go from pose i to pose j.
  Vector3f rotation;
  // Translation covariance.
  Matrix3f translation_covariance;
  // Rotation covariance, in angle-axis form.
  Matrix3f rotation_covariance;
};

struct SLAMNode {
  // Pose ID.
  uint64_t id;
  // 6DOF parameters.
  RobotPose pose;
  // Observed vision features.
  std::vector<VisionFeature> features;
  // Indicates whether this node should have any visual features or not.
  bool is_vision_node;
  // Default constructor: do nothing.
  SLAMNode() {}
  // Convenience constructor, initialize all components.
  SLAMNode(uint64_t id,
           const RobotPose& pose,
	   const std::vector<VisionFeature> feature_list,
           bool is_vision) :
      id(id), pose(pose), features(feature_list), is_vision_node(is_vision) {}
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

#endif // VIO_TYPES_H