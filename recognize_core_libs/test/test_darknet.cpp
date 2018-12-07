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

#include <iostream>
#include <string>
#include <vector>

#include <ros/package.h>

#include "gtest/gtest.h"
#include "yolo_v2_class.hpp"
#include "recognize_core_libs/utils.h"
#include "recognize_core_libs/Labels.h"
#include "recognize_core_libs/types.h"

TEST(UtilsTest, get_subimage)
{
  std::string pkgpath = ros::package::getPath("recognize_core_libs");
  Detector detector(pkgpath+"/../ThirdParty/darknet/cfg/yolov3.cfg", pkgpath+"/../ThirdParty/darknet/yolov3.weights");
  std::vector<bbox_t> d1 = detector.detect(pkgpath+"/../ThirdParty/darknet/data/dog.jpg");

  ASSERT_EQ(d1.size(), 3u);
}

TEST(UtilsTest, pipeline)
{
  std::string pkgpath = ros::package::getPath("recognize_core_libs");
  Detector detector(pkgpath+"/../ThirdParty/darknet/cfg/yolov3.cfg", pkgpath+"/../ThirdParty/darknet/yolov3.weights");
  recognize::Labels labels(pkgpath+"/../ThirdParty/darknet/data/coco.names");

  recognize::ImageDetectionInfo image_detection;

  image_detection.image = cv::imread(pkgpath+"/test/people.jpg");
  image_detection.bounding_boxes = detector.detect(image_detection.image, 0.4);

  recognize::drawBB(&image_detection, labels);

  cv::imwrite(pkgpath+"/test/bb.jpg", image_detection.image);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
