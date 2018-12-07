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

#include <vector>
#include <algorithm>

#include <cv_bridge/cv_bridge.h>

#include <recognize_msgs/ArrayBoundingBox.h>
#include <recognize_msgs/BoundingBox.h>

#include "recognize_core_libs/translations.h"
#include "recognize_core_libs/types.h"

namespace recognize
{

void transform(const bbox_t& bb, recognize_msgs::BoundingBox *msg_bb)
{
  msg_bb->x = bb.x;
  msg_bb->y = bb.y;
  msg_bb->w = bb.w;
  msg_bb->h = bb.h;
  msg_bb->prob = bb.prob;
  msg_bb->obj_id = bb.obj_id;
}

void transform(const ImageDetectionInfo& image_detection_info, recognize_msgs::ArrayBoundingBox* msg)
{
  for (int i=0; i<image_detection_info.bounding_boxes.size(); i++)
  {
    recognize_msgs::BoundingBox msg_bb;
    transform(image_detection_info.bounding_boxes[i], &msg_bb);

    msg->bounding_boxes.push_back(msg_bb);
  }
}

void transform(const ImageDetectionInfo& image_detection_info, sensor_msgs::Image *msg)
{
  cv_bridge::CvImage img_bridge;

  std_msgs::Header header;
  header.seq = 0;
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image_detection_info.image);
  img_bridge.toImageMsg(*msg);
}

void transform(const recognize_msgs::BoundingBox& msg_bb, bbox_t* bb)
{
  bb->x = msg_bb.x;
  bb->y = msg_bb.y;
  bb->w = msg_bb.w;
  bb->h = msg_bb.h;
  bb->prob = msg_bb.prob;
  bb->obj_id = msg_bb.obj_id;
}

void transform(const recognize_msgs::ArrayBoundingBox& msg, ImageDetectionInfo* image_detection_info)
{
  for (int i=0; i<msg.bounding_boxes.size(); i++)
  {
    bbox_t bb;
    transform(msg.bounding_boxes[i], &bb);

    image_detection_info->bounding_boxes.push_back(bb);
  }
}

void transform(const sensor_msgs::Image& msg, ImageDetectionInfo* image_detection_info)
{
  cv_bridge::CvImagePtr img;
  img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  image_detection_info->image = img->image;
}

}  // namespace recognize
