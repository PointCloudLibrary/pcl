/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2012-, Open Perception, Inc.

 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_FILTERS_IMPL_MEDIAN_FILTER_HPP_
#define PCL_FILTERS_IMPL_MEDIAN_FILTER_HPP_

#include <pcl/filters/median_filter.h>
#include <pcl/common/io.h>

template <typename PointT> void
pcl::MedianFilter<PointT>::applyFilter (PointCloud &output)
{
  if (!input_->isOrganized ())
  {
    PCL_ERROR ("[pcl::MedianFilter] Input cloud needs to be organized\n");
    return;
  }

  // Copy everything from the input cloud to the output cloud (takes care of all the fields)
  copyPointCloud (*input_, output);

  int height = static_cast<int> (output.height);
  int width = static_cast<int> (output.width);
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      if (pcl::isFinite ((*input_)(x, y)))
      {
        std::vector<float> vals;
        vals.reserve (window_size_ * window_size_);
        // Fill in the vector of values with the depths around the interest point
        for (int y_dev = -window_size_/2; y_dev <= window_size_/2; ++y_dev)
          for (int x_dev = -window_size_/2; x_dev <= window_size_/2; ++x_dev)
          {
            if (x + x_dev >= 0 && x + x_dev < width &&
                y + y_dev >= 0 && y + y_dev < height &&
                pcl::isFinite ((*input_)(x+x_dev, y+y_dev)))
              vals.push_back ((*input_)(x+x_dev, y+y_dev).z);
          }

        if (vals.size () == 0)
          continue;

        // The output depth will be the median of all the depths in the window
        partial_sort (vals.begin (), vals.begin () + vals.size () / 2 + 1, vals.end ());
        float new_depth = vals[vals.size () / 2];
        // Do not allow points to move more than the set max_allowed_movement_
        if (fabs (new_depth - (*input_)(x, y).z) < max_allowed_movement_)
          output (x, y).z = new_depth;
        else
          output (x, y).z = (*input_)(x, y).z +
                            max_allowed_movement_ * (new_depth - (*input_)(x, y).z) / fabsf (new_depth - (*input_)(x, y).z);
      }
}


#endif /* PCL_FILTERS_IMPL_MEDIAN_FILTER_HPP_ */
