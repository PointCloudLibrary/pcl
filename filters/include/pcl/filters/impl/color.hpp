/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  Author: Suat Gedikli (gedikli@willowgarage.com)
 *
 */

#ifndef PCL_FILTERS_IMPL_COLOR_H_
#define PCL_FILTERS_IMPL_COLOR_H_

#include "pcl/filters/color.h"
#include "pcl/common/io.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ColorFilter<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  // Check if we're going to keep the organized structure of the cloud or not
  if (keep_organized_)
  {
    output.width  = input_->width;
    output.height = input_->height;
    output.is_dense = false;
  }
  else
  {
    output.height = 1;
    output.is_dense = input_->is_dense;
  }
  output.points.resize (input_->points.size ());
  removed_indices_->resize (input_->points.size ());
  
  int nr_p = 0;
  int nr_removed_p = 0;
  float nan = std::numeric_limits<float>::quiet_NaN ();
  
  if (keep_organized_)
  {
    for (size_t cp = 0; cp < input_->points.size (); ++cp)
    {
      if (lookup_[input_->points[cp].rgba])
        output.points [cp] = input_->points[cp];
      else
      {
        output.points [cp].x = output.points[cp].y = output.points[cp].z = nan;
        // dont loose color information
        output.points [cp].rgba = input_->points[cp].rgba;
      }
    }
    nr_p = input_->points.size ();
  }
  else // Remove filtered points
  {
    for (size_t cp = 0; cp < input_->points.size (); ++cp)
    {
      if (lookup_[input_->points[cp].rgba])
        output.points [nr_p++] = input_->points[cp];
      else if (extract_removed_indices_)
          (*removed_indices_)[nr_removed_p++] = cp;
    }
    output.width = nr_p;
  } // !keep_organized_
  output.points.resize (output.width * output.height);
  removed_indices_->resize(nr_removed_p);
}

#define PCL_INSTANTIATE_ColorFilter(T) template class PCL_EXPORTS pcl::ColorFilter<T>;

#endif    // PCL_FILTERS_IMPL_COLOR_H_

