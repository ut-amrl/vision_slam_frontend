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
\file    slam_main.cc
\brief   Main entry point to process a ROS bag file through a vision SLAM
         frontend, and save the resulting SLAM dataset.
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>
#include <queue>

#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CompressedImage.h"
#include "nav_msgs/Odometry.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"
#include "cv_bridge/cv_bridge.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "vision_slam_frontend/CameraIntrinsics.h"
#include "vision_slam_frontend/CameraExtrinsics.h"

#include "gui_helpers.h"
#include "slam_frontend.h"
#include "slam_to_ros.h"


using gui_helpers::ClearMarker;
using gui_helpers::Color4f;
using gui_helpers::AddPoint;
using gui_helpers::AddLine;
using gui_helpers::InitializeMarker;
using ros::Time;
using slam::Frontend;
using slam_types::RobotPose;
using slam_types::SLAMNode;
using slam_types::SLAMProblem;
using slam_types::OdometryFactor;
using slam_types::VisionFactor;
using slam_types::FeatureMatch;
using slam_types::VisionFeature;
using std::cout;
using std::string;
using std::vector;
using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Quaternionf;
using Eigen::Translation3f;
using Eigen::Vector3f;
using Eigen::Matrix3f;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

DEFINE_string(left_image_topic,
              "/stereo/left/image_raw/compressed",
              "Name of ROS topic for left camera images");
DEFINE_string(right_image_topic,
              "/stereo/right/image_raw/compressed",
              "Name of ROS topic for right camera images");
DEFINE_string(odom_topic,
              "/odometry/filtered",
              "Name of ROS topic for odometry data");
DEFINE_string(input, "", "Name of ROS bag file to load");
DEFINE_string(output, "", "Name of ROS bag file to output");
DEFINE_bool(visualize, false, "Display images loaded");
DEFINE_bool(save_debug, false, "Save debug images");
DEFINE_int32(max_poses, 0, "Maximum number of SLAM poses to process");
DECLARE_string(helpon);
DECLARE_int32(v);

cv::Mat DecodeImage(const sensor_msgs::CompressedImage& msg) {
  cv::Mat image = cv::imdecode(cv::InputArray(msg.data),
                               cv::ImreadModes::IMREAD_GRAYSCALE);
  if (msg.format.find("bayer_rggb8") != string::npos) {
    vector<cv::Mat1b> image_channels(image.channels());
    cv::split(image, image_channels.data());
    image = image_channels[0];
    cv::cvtColor(image_channels[0], image, cv::COLOR_BayerBG2BGR);
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  }
  return image;
}

bool CompressedImageCallback(const std::pair<
                                       sensor_msgs::CompressedImage,
                                       sensor_msgs::CompressedImage>& img_pair,
                             Frontend* frontend) {
  double image_time = img_pair.first.header.stamp.toSec();
  if (FLAGS_v > 1) {
    printf("CompressedImage t=%f\n", image_time);
  }
  // Decode the left image
  // Process the left image
  // Then find the closest point in the left image.
  // Calculate the Depth and X Y and broadcast based on that.
  cv::Mat left_image = DecodeImage(img_pair.first);
  cv::Mat right_image = DecodeImage(img_pair.second);
  if (FLAGS_visualize) {
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", left_image);
    if (cv::waitKey(16) == 27) {
      exit(0);
    }
  }
  return frontend->ObserveImage(left_image, right_image, image_time);
}

void OdometryCallback(const nav_msgs::Odometry& msg,
                      Frontend* frontend) {
  if (FLAGS_v > 1) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector3f odom_loc(msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z);
  const Quaternionf  odom_angle(msg.pose.pose.orientation.w,
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z);
  frontend->ObserveOdometry(odom_loc, odom_angle, msg.header.stamp.toSec());
}

using std::isfinite;
bool IsFinite(const Vector3f& p) {
  return isfinite(p.x()) && isfinite(p.y()) && isfinite(p.z());
}

void AddFeaturePoints(const slam::FrontendConfig& config,
                      const SLAMProblem& problem,
                      Marker* marker_ptr) {
  const Affine3f cam_to_robot = config.left_cam_to_robot;
  for (const SLAMNode& node :  problem.nodes) {
    const Affine3f robot_to_world = node.pose.RobotToWorldTf();
    for (const slam_types::VisionFeature& f : node.features) {
      if (IsFinite(f.point3d)) {
        if (f.point3d.z() > 0.1 &&
            f.point3d.norm() > 0.5 &&
            f.point3d.norm() < 20.0) {
          AddPoint(robot_to_world * cam_to_robot * f.point3d,
                  Color4f(1, 1, 1, 0.2),
                  marker_ptr);
        }
      }
    }
  }
}

void AddPoseGraph(const SLAMProblem& problem,
                  Marker* nodes_marker,
                  Marker* vision_marker,
                  Marker* odom_marker) {
  for (const SLAMNode& node :  problem.nodes) {
    AddPoint(node.pose.loc, Color4f::kRed, nodes_marker);
  }
  for (const OdometryFactor& factor : problem.odometry_factors) {
    const Vector3f& loc1 = problem.nodes[factor.pose_i].pose.loc;
    const Vector3f& loc2 = problem.nodes[factor.pose_j].pose.loc;
    AddLine(loc1, loc2, Color4f::kGreen, odom_marker);
  }
  for (const VisionFactor& factor : problem.vision_factors) {
    const Vector3f& loc1 = problem.nodes[factor.pose_idx_initial].pose.loc;
    const Vector3f& loc2 = problem.nodes[factor.pose_idx_current].pose.loc;
    AddLine(loc1, loc2, Color4f::kBlue, vision_marker);
  }
}

void PublishVisualization(const slam::FrontendConfig& config,
                          const SLAMProblem& problem,
                          ros::Publisher* graph_publisher,
                          ros::Publisher* point_cloud_publisher) {
  Marker nodes_marker;
  Marker odom_marker;
  Marker vision_marker;
  Marker vision_points_marker;
  InitializeMarker(
      Marker::POINTS, Color4f::kRed, 0.05, 0.1, 0, &nodes_marker);
  InitializeMarker(
      Marker::LINE_LIST, Color4f::kGreen, 0.02, 0, 0, &odom_marker);
  InitializeMarker(
      Marker::LINE_LIST, Color4f::kBlue, 0.01, 0, 0, &vision_marker);
  InitializeMarker(
      Marker::POINTS, Color4f::kWhite, 0.025, 0.025, 0.025,
      &vision_points_marker);
  nodes_marker.id = 0;
  odom_marker.id = 1;
  vision_marker.id = 2;
  vision_points_marker.id = 3;

  AddFeaturePoints(config, problem, &vision_points_marker);
  AddPoseGraph(problem, &nodes_marker, &vision_marker, &odom_marker);

  MarkerArray markers;
  markers.markers.push_back(nodes_marker);
  markers.markers.push_back(odom_marker);
  markers.markers.push_back(vision_marker);
  graph_publisher->publish(markers);
  point_cloud_publisher->publish(vision_points_marker);
}

// Comparator used to determine left and right frame pairs.
class SeqCompare {
 public:
  bool operator()(sensor_msgs::CompressedImage a,
                  sensor_msgs::CompressedImage b) {
    return a.header.seq < b.header.seq;
  }
};

void ProcessBagfile(const char* filename, ros::NodeHandle* n) {
  rosbag::Bag bag;
  cv_bridge::CvImage img_tranform;
  try {
    bag.open(filename, rosbag::bagmode::Read);
  } catch(rosbag::BagException exception) {
    printf("Unable to read %s, reason:\n %s\n", filename, exception.what());
    return;
  }
  printf("Processing %s\n", filename);
  vector<string> topics;
  topics.push_back(FLAGS_left_image_topic.c_str());
  topics.push_back(FLAGS_right_image_topic.c_str());
  topics.push_back(FLAGS_odom_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  slam::Frontend slam_frontend("");
  ros::Publisher gui_publisher = n->advertise<visualization_msgs::MarkerArray>(
      "slam_frontend/pose_graph", 1);
  ros::Publisher debug_image_publisher = n->advertise<sensor_msgs::Image>(
      "slam_frontend/debug_image", 1);
  ros::Publisher debug_stereo_image_publisher =
      n->advertise<sensor_msgs::Image>("slam_frontend/debug_stereo_image", 1);
  ros::Publisher point_cloud_publisher =
      n->advertise<visualization_msgs::Marker>("slam_frontend/points", 1);
  double bag_t_start = -1;
  bool max_poses_processed = false;
  std::pair<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
      curr_stereo_image_pair;
  // Iterate for every message.
  std::priority_queue<sensor_msgs::CompressedImage,
                      std::vector<sensor_msgs::CompressedImage>,
                      SeqCompare> left_queue;
  std::priority_queue<sensor_msgs::CompressedImage,
                      std::vector<sensor_msgs::CompressedImage>,
                      SeqCompare> right_queue;
  for (rosbag::View::iterator it = view.begin();
       ros::ok() && it != view.end() && !max_poses_processed;
       ++it) {
    const rosbag::MessageInstance& message = *it;
    bool new_pose_added = false;
    if (bag_t_start < 0.0) {
      bag_t_start = message.getTime().toSec();
    }
    {
      sensor_msgs::CompressedImagePtr image_msg =
          message.instantiate<sensor_msgs::CompressedImage>();
      if (image_msg != NULL) {
        const bool is_left_camera =
           message.getTopic() == FLAGS_left_image_topic;
        new_pose_added = false;
        if (is_left_camera) {
          curr_stereo_image_pair.first = *image_msg;
        } else {
          curr_stereo_image_pair.second = *image_msg;
          CHECK_EQ(curr_stereo_image_pair.first.header.stamp.toNSec(),
                   curr_stereo_image_pair.second.header.stamp.toNSec());
          new_pose_added = CompressedImageCallback(curr_stereo_image_pair,
                                                   &slam_frontend);
        }
        if (new_pose_added) {
          if (FLAGS_max_poses > 0 &&
              slam_frontend.GetNumPoses() > FLAGS_max_poses) {
            max_poses_processed = true;
          }
          static std_msgs::Header debug_image_header;
          debug_image_header.seq++;
          debug_image_header.stamp = ros::Time::now();
          img_tranform.image = slam_frontend.GetLastDebugImage();
          img_tranform.encoding = sensor_msgs::image_encodings::RGB8;
          debug_image_publisher.publish(img_tranform.toImageMsg());
          img_tranform.image = slam_frontend.GetLastDebugStereoImage();
          img_tranform.encoding = sensor_msgs::image_encodings::RGB8;
          debug_stereo_image_publisher.publish(img_tranform.toImageMsg());
        }
      }
    }
    {
      nav_msgs::OdometryPtr odom_msg =
          message.instantiate<nav_msgs::Odometry>();
      if (odom_msg != NULL) {
        OdometryCallback(*odom_msg, &slam_frontend);
      }
    }
    if (new_pose_added) {
      SLAMProblem problem;
      slam_frontend.GetSLAMProblem(&problem);
      PublishVisualization(slam_frontend.GetConfig(),
                           problem,
                           &gui_publisher,
                           &point_cloud_publisher);
      ros::spinOnce();
    }
  }
  printf("Done processing bag file.\n");
  // Publish slam data
  rosbag::Bag output_bag;
  try {
    output_bag.open(FLAGS_output.c_str(), rosbag::bagmode::Write);
  } catch(rosbag::BagException exception) {
    printf("Unable to open %s, reason:\n %s\n",
            FLAGS_output.c_str(),
            exception.what());
    return;
  }
  
  slam_types::CameraExtrinsics a;
  const Affine3f extrinsics = slam_frontend.GetConfig().left_cam_to_robot;
  const Vector3f rT = extrinsics.translation();
  const AngleAxisf rR(extrinsics.rotation());

  for (int i = 0; i < 3; ++i) {
    a.translation[i] = rT[i];
    if (rR.angle() > 1e-8) {
      a.rotation[i] = rR.axis().normalized()[i] * rR.angle();
    } else {
      a.rotation[i] = 0;
    }
  }
  output_bag.write<vision_slam_frontend::CameraExtrinsics>(
    "extrinsics",
    ros::Time::now(),
    ExtrinsicsToRos(a));
  vision_slam_frontend::CameraIntrinsics k;
  k.fx = slam_frontend.GetConfig().intrinsics_left.fx;
  k.cx = slam_frontend.GetConfig().intrinsics_left.cx;
  k.fy = slam_frontend.GetConfig().intrinsics_left.fy;
  k.cy = slam_frontend.GetConfig().intrinsics_left.cy;

  output_bag.write<vision_slam_frontend::CameraIntrinsics>(
      "intrinsics",
      ros::Time::now(),
      k);
  
  SLAMProblem problem;
  slam_frontend.GetSLAMProblem(&problem);
  output_bag.write<vision_slam_frontend::SLAMProblem>(
      "slam_problem",
      ros::Time::now(),
      SLAMProblemToRos(problem));
  printf("Saved SLAM problem with %d nodes, %d odometry factors, "
  "%d vision factors (%.2f/pose avg)\n",
         static_cast<int>(problem.nodes.size()),
         static_cast<int>(problem.odometry_factors.size()),
         static_cast<int>(problem.vision_factors.size()),
         static_cast<float>(problem.vision_factors.size()) /
         static_cast<float>((problem.nodes.size() - 1)));
  if (FLAGS_save_debug) {
    {
      std::vector<cv::Mat> debug_images = slam_frontend.getDebugImages();
      uint64_t count = 0;
      for (auto im : debug_images) {
        std_msgs::Header h;
        h.seq = count++;
        h.stamp = ros::Time::now();
        img_tranform.image = im;
        img_tranform.encoding = sensor_msgs::image_encodings::RGB8;
        output_bag.write<sensor_msgs::Image>("slam_debug_images",
                                              ros::Time::now(),
                                              img_tranform.toImageMsg());
      }
    }
    {
      std::vector<cv::Mat> debug_stereo_images =
          slam_frontend.getDebugStereoImages();
      uint64_t count = 0;
      for (auto im : debug_stereo_images) {
        printf("%d\n", im.flags);
        std_msgs::Header h;
        h.seq = count++;
        h.stamp = ros::Time::now();
        img_tranform.image = im;
        img_tranform.encoding = sensor_msgs::image_encodings::RGB8;
        output_bag.write<sensor_msgs::Image>("slam_debug_stereo_images",
                                              ros::Time::now(),
                                              img_tranform.toImageMsg());
      }
    }
  }
  output_bag.close();
}

void SignalHandler(int signum) {
  printf("Exiting with signal %d\n", signum);
  exit(0);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  if (FLAGS_input == "") {
    fprintf(stderr, "ERROR: Must specify input file!\n");
    return 1;
  }
  if (FLAGS_output == "") {
    fprintf(stderr, "ERROR: Must specify output file!\n");
    return 1;
  }
  // Initialize ROS.
  ros::init(argc, argv, "slam_frontend");
  ros::NodeHandle n;
  signal(SIGINT, SignalHandler);
  ProcessBagfile(FLAGS_input.c_str(), &n);
  return 0;
}
