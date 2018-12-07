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
#include <vector>
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>

#include <recognize_msgs/ArrayBoundingBox.h>
#include <recognize_msgs/BoundingBox.h>

#include "gtest/gtest.h"
#include "recognize_core_libs/utils.h"
#include "recognize_core_libs/types.h"
#include "recognize_core_libs/translations.h"

TEST(UtilsTest, transforms_bb_msg)
{
  bbox_t bb1;

  bb1.x = 10;
  bb1.y = 20;
  bb1.w = 300;
  bb1.h = 200;
  bb1.prob = 0.3;
  bb1.obj_id = 2;

  recognize_msgs::BoundingBox msg_bb1;

  recognize::transform(bb1, &msg_bb1);

  ASSERT_EQ(bb1.x, msg_bb1.x);
  ASSERT_EQ(bb1.y, msg_bb1.y);
  ASSERT_EQ(bb1.w, msg_bb1.w);
  ASSERT_EQ(bb1.h, msg_bb1.h);
  ASSERT_EQ(bb1.prob, msg_bb1.prob);
  ASSERT_EQ(bb1.obj_id, msg_bb1.obj_id);

  bbox_t bb2;
  recognize::transform(msg_bb1, &bb2);

  ASSERT_EQ(bb2.x, msg_bb1.x);
  ASSERT_EQ(bb2.y, msg_bb1.y);
  ASSERT_EQ(bb2.w, msg_bb1.w);
  ASSERT_EQ(bb2.h, msg_bb1.h);
  ASSERT_EQ(bb2.prob, msg_bb1.prob);
  ASSERT_EQ(bb2.obj_id, msg_bb1.obj_id);

  ASSERT_EQ(bb1.x, bb2.x);
  ASSERT_EQ(bb1.y, bb2.y);
  ASSERT_EQ(bb1.w, bb2.w);
  ASSERT_EQ(bb1.h, bb2.h);
  ASSERT_EQ(bb1.prob, bb2.prob);
  ASSERT_EQ(bb1.obj_id, bb2.obj_id);
}

TEST(UtilsTest, transforms_bb_arraymsg)
{
  recognize::ImageDetectionInfo image_detection_info;
  bbox_t bb1, bb2;

  bb1.x = 10;
  bb1.y = 20;
  bb1.w = 300;
  bb1.h = 200;
  bb1.prob = 0.3;
  bb1.obj_id = 2;

  bb2.x = 15;
  bb2.y = 25;
  bb2.w = 305;
  bb2.h = 205;
  bb2.prob = 0.5;
  bb2.obj_id = 5;

  image_detection_info.bounding_boxes.push_back(bb1);
  image_detection_info.bounding_boxes.push_back(bb2);

  recognize_msgs::ArrayBoundingBox abb;

  recognize::transform(image_detection_info, &abb);

  recognize::ImageDetectionInfo image_detection_info2;

  recognize::transform(abb, &image_detection_info2);

  ASSERT_EQ(image_detection_info.bounding_boxes[0].x, image_detection_info2.bounding_boxes[0].x);
  ASSERT_EQ(image_detection_info.bounding_boxes[0].y, image_detection_info2.bounding_boxes[0].y);
  ASSERT_EQ(image_detection_info.bounding_boxes[0].w, image_detection_info2.bounding_boxes[0].w);
  ASSERT_EQ(image_detection_info.bounding_boxes[0].h, image_detection_info2.bounding_boxes[0].h);
  ASSERT_EQ(image_detection_info.bounding_boxes[0].prob, image_detection_info2.bounding_boxes[0].prob);
  ASSERT_EQ(image_detection_info.bounding_boxes[0].obj_id, image_detection_info2.bounding_boxes[0].obj_id);
  ASSERT_EQ(image_detection_info.bounding_boxes[1].x, image_detection_info2.bounding_boxes[1].x);
  ASSERT_EQ(image_detection_info.bounding_boxes[1].y, image_detection_info2.bounding_boxes[1].y);
  ASSERT_EQ(image_detection_info.bounding_boxes[1].w, image_detection_info2.bounding_boxes[1].w);
  ASSERT_EQ(image_detection_info.bounding_boxes[1].h, image_detection_info2.bounding_boxes[1].h);
  ASSERT_EQ(image_detection_info.bounding_boxes[1].prob, image_detection_info2.bounding_boxes[1].prob);
  ASSERT_EQ(image_detection_info.bounding_boxes[1].obj_id, image_detection_info2.bounding_boxes[1].obj_id);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
