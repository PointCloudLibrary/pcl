/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#include <pcl/recognition/mask_map.h>

#include <algorithm> // for std::transform
#include <cassert>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::MaskMap::MaskMap(const std::size_t width, const std::size_t height)
: data_(width * height), width_(width), height_(height)
{}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::MaskMap::resize(const std::size_t width, const std::size_t height)
{
  data_.resize(width * height);
  width_ = width;
  height_ = height;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::MaskMap
pcl::MaskMap::getDifferenceMask(const MaskMap& mask0, const MaskMap& mask1)
{
  assert(mask0.getWidth() == mask1.getWidth());
  assert(mask0.getHeight() == mask1.getHeight());

  pcl::MaskMap diff_mask{mask0.getWidth(), mask0.getHeight()};

  std::transform(std::cbegin(mask0.data_),
                 std::cend(mask0.data_),
                 std::cbegin(mask1.data_),
                 std::begin(diff_mask.data_),
                 [](const char& b0, const char& b1) { return b0 == b1 ? 0 : 255; });

  return diff_mask;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::MaskMap::erode(MaskMap& eroded_mask) const
{
  const MaskMap& mask_in = *this;
  eroded_mask.resize(width_, height_);

  for (std::size_t row_index = 1; row_index < height_ - 1; ++row_index) {
    for (std::size_t col_index = 1; col_index < width_ - 1; ++col_index) {
      if (!mask_in.isSet(col_index, row_index - 1) ||
          !mask_in.isSet(col_index - 1, row_index) ||
          !mask_in.isSet(col_index + 1, row_index) ||
          !mask_in.isSet(col_index, row_index + 1)) {
        eroded_mask.unset(col_index, row_index);
      }
      else {
        eroded_mask.set(col_index, row_index);
      }
    }
  }
}
