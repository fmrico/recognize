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

#include <exception>
#include <vector>
#include <algorithm>

#include "recognize_core_libs/utils.h"
#include "recognize_core_libs/types.h"

namespace recognize
{

void drawBB(ImageDetectionInfo* image_detection_info, const Labels& labels)
{
  for (unsigned int i = 0; i < image_detection_info->bounding_boxes.size(); i++)
  {
    cv::Rect dst_rect_roi(cv::Point2i(image_detection_info->bounding_boxes[i].x,
      image_detection_info->bounding_boxes[i].y),
      cv::Size2i(image_detection_info->bounding_boxes[i].w, image_detection_info->bounding_boxes[i].h));

    putText(image_detection_info->image, labels.get_label(image_detection_info->bounding_boxes[i].obj_id) +
      " "+ std::to_string(image_detection_info->bounding_boxes[i].prob),
      dst_rect_roi.tl() - cv::Point2i(-4, 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.9, cv::Scalar(0, 0, 0), 2);

    cv::rectangle(image_detection_info->image, dst_rect_roi, cv::Scalar(1, 0, 1), 5);
  }
}

}  // namespace recognize
