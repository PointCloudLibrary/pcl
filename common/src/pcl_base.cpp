/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <algorithm>
#include <numeric>

#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/io.h>  // for getFieldSize

///////////////////////////////////////////////////////////////////////////////////////////
pcl::PCLBase<pcl::PCLPointCloud2>::PCLBase ()
  : use_indices_ (false)
  , fake_indices_ (false)
  , field_sizes_ (0)
  , x_idx_ (UNAVAILABLE)
  , y_idx_ (UNAVAILABLE)
  , z_idx_ (UNAVAILABLE)
  , x_field_name_ ("x")
  , y_field_name_ ("y")
  , z_field_name_ ("z")
{
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PCLBase<pcl::PCLPointCloud2>::setInputCloud (const PCLPointCloud2ConstPtr &cloud)
{
  input_ = cloud;

  for (std::size_t d = 0; d < cloud->fields.size (); ++d)
  {
    if (cloud->fields[d].name == x_field_name_)
      x_idx_ = d;
    if (cloud->fields[d].name == y_field_name_)
      y_idx_ = d;
    if (cloud->fields[d].name == z_field_name_)
      z_idx_ = d;
  }

  // Obtain the size of datatype
  const auto sizeofDatatype = [](const auto& datatype) -> int
  {
    const auto size = getFieldSize(datatype);
    if (size == 0) {
        PCL_ERROR("[PCLBase::setInputCloud] Invalid field type (%d)!\n", datatype);
    }
    return size;
  };

  // Restrict size of a field to be at-max sizeof(FLOAT64) now to support {U}INT64
  field_sizes_.resize(input_->fields.size());
  std::transform(input_->fields.begin(), input_->fields.end(), field_sizes_.begin(),
                 [&sizeofDatatype](const auto& field)
                 {
                   return std::min(sizeofDatatype(field.datatype), static_cast<int>(sizeof(double)));
                 });
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::PCLBase<pcl::PCLPointCloud2>::deinitCompute ()
{
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::PCLBase<pcl::PCLPointCloud2>::initCompute ()
{
  // Check if input was set
  if (!input_)
    return (false);

  // If no point indices have been given, construct a set of indices for the entire input point cloud
  if (!indices_)
  {
    fake_indices_ = true;
    indices_.reset (new Indices);
  }

  // If we have a set of fake indices, but they do not match the number of points in the cloud, update them
  if (fake_indices_ && indices_->size () != (input_->width * input_->height))
  {
    const auto indices_size = indices_->size ();
    try
    {
      indices_->resize (input_->width * input_->height);
    }
    catch (const std::bad_alloc&)
    {
      PCL_ERROR ("[initCompute] Failed to allocate %lu indices.\n", (input_->width * input_->height));
    }
    if (indices_size < indices_->size ())
      std::iota(indices_->begin () + indices_size, indices_->end (), indices_size);
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PCLBase<pcl::PCLPointCloud2>::setIndices (const IndicesPtr &indices)
{
  indices_ = indices;
  fake_indices_ = false;
  use_indices_  = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PCLBase<pcl::PCLPointCloud2>::setIndices (const PointIndicesConstPtr &indices)
{
  indices_.reset (new Indices(indices->indices));
  fake_indices_ = false;
  use_indices_  = true;
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(PCLBase, PCL_POINT_TYPES)
#endif    // PCL_NO_PRECOMPILE

