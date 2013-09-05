/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 *  keypoint.h
 *
 *  Created on: May 28, 2012
 *      Author: somani
 */

#ifndef PCL_2D_KEYPOINT_H_
#define PCL_2D_KEYPOINT_H_

#include <pcl/2d/edge.h>

namespace pcl
{
  class Keypoint
  {
    private:
      Edge edge_detection;
      Convolution conv_2d;
    public:
      Keypoint  ()
      {
      }
      
      void 
      harrisCorner  (ImageType &output, ImageType &input, const float sigma_d, const float sigma_i, const float alpha, const float thresh);
      
      void 
      hessianBlob  (ImageType &output, ImageType &input, const float sigma, bool SCALE);
      
      void 
      hessianBlob  (ImageType &output, ImageType &input, const float start_scale, const float scaling_factor, const int num_scales);

      void 
      imageElementMultiply  (ImageType &output, ImageType &input1, ImageType &input2);
  };
}

#include <pcl/2d/impl/keypoint.hpp>

#endif    // PCL_2D_KEYPOINT_H_
