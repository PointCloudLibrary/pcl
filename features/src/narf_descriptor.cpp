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
 */

#include <iostream>
using std::cout;
using std::cerr;

#include <pcl/features/narf_descriptor.h>
#include <pcl/features/narf.h>
#include <pcl/range_image/range_image.h>

namespace pcl 
{
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
NarfDescriptor::NarfDescriptor (const RangeImage* range_image, const std::vector<int>* indices) : 
  BaseClass (), range_image_ (), parameters_ ()
{
  setRangeImage (range_image, indices);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
NarfDescriptor::~NarfDescriptor ()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
NarfDescriptor::setRangeImage (const RangeImage* range_image, const std::vector<int>* indices)
{
  range_image_ = range_image;
  if (indices != NULL)
  {
    IndicesPtr indicesptr (new std::vector<int> (*indices));
    setIndices (indicesptr);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
NarfDescriptor::computeFeature(NarfDescriptor::PointCloudOut& output)
{
  //std::cout << __PRETTY_FUNCTION__ << " called.\n";
  
  output.points.clear();
  
  if (range_image_==NULL)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": RangeImage is not set. Sorry, the NARF descriptor calculation works on range images, not on normal point clouds."
              << " Use setRangeImage(...).\n\n";
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (parameters_.support_size <= 0.0f)
  {
    std::cerr << __PRETTY_FUNCTION__
              << ": support size is not set. Use getParameters().support_size = ...\n\n";
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  std::vector<Narf*> feature_list;
  if (indices_)
  {
    for (size_t indices_idx=0; indices_idx<indices_->size(); ++indices_idx)
    {
      int point_index = (*indices_)[indices_idx];
      int y=point_index/range_image_->width, x=point_index - y*range_image_->width;
      Narf::extractFromRangeImageAndAddToList(*range_image_, static_cast<float> (x), static_cast<float> (y), 36, parameters_.support_size,
                                              parameters_.rotation_invariant, feature_list);
    }
  }
  else
  {
    for (unsigned int y=0; y<range_image_->height; ++y)
    {
      for (unsigned int x=0; x<range_image_->width; ++x)
      {
        Narf::extractFromRangeImageAndAddToList(*range_image_, static_cast<float> (x), static_cast<float> (y), 36, parameters_.support_size,
                                                parameters_.rotation_invariant, feature_list);
      }
    }
  }
  
  // Copy to NARF36 struct
  output.points.resize(feature_list.size());
  for (unsigned int i=0; i<feature_list.size(); ++i)
  {
    feature_list[i]->copyToNarf36(output.points[i]);
  }
  
  // Cleanup
  for (size_t i=0; i<feature_list.size(); ++i)
    delete feature_list[i];
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
NarfDescriptor::compute(NarfDescriptor::PointCloudOut& output)
{
  computeFeature(output);
}
}  // namespace end
