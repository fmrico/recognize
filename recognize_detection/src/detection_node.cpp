/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Intelligent Robotics Lab URJC
 *  Copyright (c) 2018, Robotics Lab UA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Robotics Labs URJC/UA nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/

/* Author: Francisco Martín Rico - fmrico@gmail.com */

#include <string>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>

#include "recognize_capture/ROSImageProvider.h"
#include "recognize_capture/CameraImageProvider.h"

#include <recognize_core_libs/Labels.h>
#include <recognize_core_libs/types.h>
#include <recognize_core_libs/utils.h>
#include <recognize_core_libs/translations.h>
#include <recognize_msgs/ArrayBoundingBox.h>
#include <yolo_v2_class.hpp>


class ImageDetector
{
 public:
    ImageDetector()
    : nh_("~"),
      it_(nh_),
      seq_(0),
      detection_threshold_(0.8)
    {
      // Default values
      std::string pkgpath = ros::package::getPath("recognize_core_libs");
      std::string config_file = pkgpath + "/../ThirdParty/darknet/cfg/yolov3.cfg";
      std::string weight_file = pkgpath + "/../ThirdParty/darknet/cfg/yolov3.weights";
      std::string labels_file = pkgpath + "/../ThirdParty/darknet/data/yolov3.labels";

      nh_.param<std::string>("yolo_config_file", config_file, config_file);
      nh_.param<std::string>("yolo_weight_file", weight_file, weight_file);
      nh_.param<std::string>("yolo_labels_file", labels_file, labels_file);
      nh_.param<double>("detection_threshold", detection_threshold_, detection_threshold_);

      detector_ = std::shared_ptr<Detector>(new Detector(config_file, weight_file));
      labels_ = std::shared_ptr<recognize::Labels>(new recognize::Labels(labels_file));

      bb_pub_ = nh_.advertise<recognize_msgs::ArrayBoundingBox>("/bounding_boxes", 100);
      detected_image_pub_ = it_.advertise("/detection_image", 100);
    }

    void publish_bounding_boxes(const recognize::ImageDetectionInfo& image_detection_info, uint32_t seq)
    {
      if (bb_pub_.getNumSubscribers() > 0)
      {
        recognize_msgs::ArrayBoundingBox msg;
        recognize::transform(image_detection_info, &msg);

        msg.header.stamp = ros::Time::now();
        msg.header.seq = seq;

        bb_pub_.publish(msg);
      }
    }

    void publish_detected_image(const recognize::ImageDetectionInfo& image_detection_info, uint32_t seq)
    {
      if (detected_image_pub_.getNumSubscribers() > 0)
      {
        sensor_msgs::Image msg;
        recognize::transform(image_detection_info, &msg);

        msg.header.stamp = ros::Time::now();
        msg.header.seq = seq;

        detected_image_pub_.publish(msg);
      }
    }

    void detect(cv::Mat* image_in)
    {
      recognize::ImageDetectionInfo image_detection_info;
      image_detection_info.image = *image_in;

      image_detection_info.bounding_boxes = detector_->detect(image_detection_info.image, detection_threshold_);

      recognize::drawBB(&image_detection_info, *labels_);

      publish_bounding_boxes(image_detection_info, seq_);
      publish_detected_image(image_detection_info, seq_);

      seq_++;
    }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  std::shared_ptr<Detector> detector_;
  std::shared_ptr<recognize::Labels> labels_;

  ros::Publisher bb_pub_;
  image_transport::Publisher detected_image_pub_;

  uint32_t seq_;
  double detection_threshold_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "detection");
  ros::NodeHandle nh("~");

  ImageDetector detector;

  std::string capture_mode("ROS");
  double rate = 1.0;

  nh.param<std::string>("capture_mode", capture_mode, capture_mode);
  nh.param<double>("rate", rate, rate);

  std::shared_ptr<recognize::ImageProvider> image_provider;
  if (capture_mode == "ROS")
    image_provider = std::shared_ptr<recognize::ImageProvider>(new recognize::ROSImageProvider());
  else if (capture_mode == "OPENCV")
    image_provider = std::shared_ptr<recognize::ImageProvider>(new recognize::CameraImageProvider());

  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    cv::Mat image_in = image_provider->get_last_image();

    if (!image_in.empty())
    {
      detector.detect(&image_in);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
