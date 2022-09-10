/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: filter.hpp 1800 2011-07-15 11:45:31Z marton $
 *
 */

#ifndef PCL_FILTERS_IMPL_FILTER_INDICES_H_
#define PCL_FILTERS_IMPL_FILTER_INDICES_H_

#include <pcl/filters/filter_indices.h>

template <typename PointT> void
pcl::removeNaNFromPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                              Indices &index)
{
  // Reserve enough space for the indices
  index.resize (cloud_in.size ());

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (int j = 0; j < static_cast<int> (cloud_in.size ()); ++j)
      index[j] = j;
  }
  else
  {
    int j = 0;
    for (int i = 0; i < static_cast<int> (cloud_in.size ()); ++i)
    {
      if (!std::isfinite (cloud_in[i].x) || 
          !std::isfinite (cloud_in[i].y) || 
          !std::isfinite (cloud_in[i].z))
        continue;
      index[j] = i;
      j++;
    }
    if (j != static_cast<int> (cloud_in.size ()))
    {
      // Resize to the correct size
      index.resize (j);
    }
  }
}

template<typename PointT> void
pcl::FilterIndices<PointT>::applyFilter (PointCloud &output)
{
  Indices indices;
  if (keep_organized_)
  {
    if (!extract_removed_indices_)
    {
      PCL_WARN ("[pcl::FilterIndices<PointT>::applyFilter] extract_removed_indices_ was set to 'true' to keep the point cloud organized.\n");
      extract_removed_indices_ = true;
    }
    applyFilter (indices);

    output = *input_;

    // To preserve legacy behavior, only coordinates xyz are filtered.
    // Copying a PointXYZ initialized with the user_filter_value_ into a generic
    // PointT, ensures only the xyz coordinates, if they exist at destination,
    // are overwritten.
    const PointXYZ ufv (user_filter_value_, user_filter_value_, user_filter_value_);
    for (const auto ri : *removed_indices_)  // ri = removed index
      copyPoint(ufv, output[ri]);
    if (!std::isfinite (user_filter_value_))
      output.is_dense = false;
  }
  else
  {
    output.is_dense = true;
    applyFilter (indices);
    pcl::copyPointCloud (*input_, indices, output);
  }
}


#define PCL_INSTANTIATE_removeNanFromPointCloud(T) template PCL_EXPORTS void pcl::removeNaNFromPointCloud<T>(const pcl::PointCloud<T>&, Indices&);
#define PCL_INSTANTIATE_FilterIndices(T) template class PCL_EXPORTS  pcl::FilterIndices<T>;

#endif    // PCL_FILTERS_IMPL_FILTER_INDICES_H_

