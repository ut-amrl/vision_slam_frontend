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
\brief   A Rosbag image extractor
\author  John Bachman, (C) 2019
*/
//========================================================================

#include <stdlib.h>
#include <string>
#include <stdint.h>

#include <ros/ros.h>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "sensor_msgs/CompressedImage.h"

DEFINE_string(input, "data/outdoor.bag", "Name of the ROS bag to have images extracted");
DEFINE_string(image_topic, "/stereo/left/image_raw/compressed", "Name of the ROS image topic to extract");
DEFINE_string(output, "data/raw_images", "Output directory for images");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  if (FLAGS_input == "") {
    fprintf(stderr, "ERROR: Must specify input file!\n");
    return 1;
  }

  google::InitGoogleLogging(*argv);

  // Initialize ROS.
  ros::init(argc, argv, "slam_frontend");
  ros::NodeHandle n;

  //Open bagDEFINE_output
  rosbag::Bag bag;
  try {
      bag.open(FLAGS_input, rosbag::bagmode::Read);
  } catch(rosbag::BagException exception) {
      LOG(ERROR) << "Unable to open " << FLAGS_input;
      exit(1);
  }

  std::vector<std::string> image_topic;
  image_topic.push_back(FLAGS_image_topic.c_str());

  uint64_t image_num = 0;
  rosbag::View view(bag, rosbag::TopicQuery(image_topic));
  std::cout << "Beginning bag image extraction on topic: "
            << FLAGS_image_topic << std::endl;
  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    const rosbag::MessageInstance& message = *it;
    ros::spinOnce();
    sensor_msgs::CompressedImagePtr image_msg =
        message.instantiate<sensor_msgs::CompressedImage>();
    if (image_msg != NULL) {
      cv::Mat image = cv::imdecode(cv::InputArray(image_msg->data), 1);
      if (image_msg->format.find("bayer_rggb8") != std::string::npos) {
        cv::Mat1b *image_channels = new cv::Mat1b[image.channels()];
        cv::split(image, image_channels);
        image = image_channels[0];
        cv::cvtColor(image_channels[0], image, cv::COLOR_BayerBG2BGR);
        delete [] image_channels;
      }
      std::stringstream ss;
      ss << FLAGS_output << "/image"
         << std::setfill('0')
         << std::setw(log(std::distance<rosbag::View::iterator>(view.begin(),
              view.end())))
         << image_num++
         << ".jpg";
      std::string path = ss.str();
      std::cout << "Writing " << path << std::endl;
      cv::imwrite(path, image);
    }
  }

  return 0;
}
