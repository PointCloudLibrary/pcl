/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#pragma once

#include <pcl/2d/convolution.h>

namespace pcl {

template <typename PointT>
void
Convolution<PointT>::filter(pcl::PointCloud<PointT>& output)
{
  int input_row = 0;
  int input_col = 0;
  // default boundary option : zero padding
  output = *input_;

  int iw = static_cast<int>(input_->width), ih = static_cast<int>(input_->height),
      kw = static_cast<int>(kernel_.width), kh = static_cast<int>(kernel_.height);
  switch (boundary_options_) {
  default:
  case BOUNDARY_OPTION_CLAMP: {
    for (int i = 0; i < ih; i++) {
      for (int j = 0; j < iw; j++) {
        float intensity = 0;
        for (int k = 0; k < kh; k++) {
          for (int l = 0; l < kw; l++) {
            int ikkh = i + k - kh / 2, jlkw = j + l - kw / 2;
            if (ikkh < 0)
              input_row = 0;
            else if (ikkh >= ih)
              input_row = ih - 1;
            else
              input_row = ikkh;

            if (jlkw < 0)
              input_col = 0;
            else if (jlkw >= iw)
              input_col = iw - 1;
            else
              input_col = jlkw;

            intensity +=
                kernel_(l, k).intensity * (*input_)(input_col, input_row).intensity;
          }
        }
        output(j, i).intensity = intensity;
      }
    }
    break;
  }

  case BOUNDARY_OPTION_MIRROR: {
    for (int i = 0; i < ih; i++) {
      for (int j = 0; j < iw; j++) {
        float intensity = 0;
        for (int k = 0; k < kh; k++) {
          for (int l = 0; l < kw; l++) {
            int ikkh = i + k - kh / 2, jlkw = j + l - kw / 2;
            if (ikkh < 0)
              input_row = -ikkh - 1;
            else if (ikkh >= ih)
              input_row = 2 * ih - 1 - ikkh;
            else
              input_row = ikkh;

            if (jlkw < 0)
              input_col = -jlkw - 1;
            else if (jlkw >= iw)
              input_col = 2 * iw - 1 - jlkw;
            else
              input_col = jlkw;

            intensity +=
                kernel_(l, k).intensity * ((*input_)(input_col, input_row).intensity);
          }
        }
        output(j, i).intensity = intensity;
      }
    }
    break;
  }

  case BOUNDARY_OPTION_ZERO_PADDING: {
    for (int i = 0; i < ih; i++) {
      for (int j = 0; j < iw; j++) {
        float intensity = 0;
        for (int k = 0; k < kh; k++) {
          for (int l = 0; l < kw; l++) {
            int ikkh = i + k - kh / 2, jlkw = j + l - kw / 2;
            if (ikkh < 0 || ikkh >= ih || jlkw < 0 || jlkw >= iw)
              continue;
            intensity += kernel_(l, k).intensity * ((*input_)(jlkw, ikkh).intensity);
          }
        }
        output(j, i).intensity = intensity;
      }
    }
    break;
  }
  } // switch
}
} // namespace pcl
