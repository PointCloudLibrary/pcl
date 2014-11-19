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

#include <pcl/impl/pcl_base.hpp>

///////////////////////////////////////////////////////////////////////////////////////////
pcl::PCLBase<pcl::PCLPointCloud2>::PCLBase ()
  : input_ ()
  , indices_ ()
  , use_indices_ (false)
  , fake_indices_ (false)
  , field_sizes_ (0)
  , x_idx_ (-1)
  , y_idx_ (-1)
  , z_idx_ (-1)
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

  for (int d = 0; d < static_cast<int>(cloud->fields.size ()); ++d)
  {
    if (cloud->fields[d].name == x_field_name_)
      x_idx_ = d;
    if (cloud->fields[d].name == y_field_name_)
      y_idx_ = d;
    if (cloud->fields[d].name == z_field_name_)
      z_idx_ = d;
  }

  // Obtain the size of all fields. Restrict to sizeof FLOAT32 for now
  field_sizes_.resize (input_->fields.size ());
  for (size_t d = 0; d < input_->fields.size (); ++d)
  {
    int fsize;
    switch (input_->fields[d].datatype)
    {
      case pcl::PCLPointField::INT8:
      case pcl::PCLPointField::UINT8:
      {
        fsize = 1;
        break;
      }

      case pcl::PCLPointField::INT16:
      case pcl::PCLPointField::UINT16:
      {
        fsize = 2;
        break;
      }

      case pcl::PCLPointField::INT32:
      case pcl::PCLPointField::UINT32:
      case pcl::PCLPointField::FLOAT32:
      {
        fsize = 4;
        break;
      }

      case pcl::PCLPointField::FLOAT64:
      {
        fsize = 8;
        break;
      }

      default:
      {
        PCL_ERROR ("[PCLBase::setInputCloud] Invalid field type (%d)!\n", input_->fields[d].datatype);
        fsize = 0;
        break;
      }
    }
    field_sizes_[d] = (std::min) (fsize, static_cast<int>(sizeof (float)));
  }
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
    indices_.reset (new std::vector<int>);
    try
    {
      indices_->resize (input_->width * input_->height);
    }
    catch (std::bad_alloc)
    {
      PCL_ERROR ("[initCompute] Failed to allocate %lu indices.\n", (input_->width * input_->height));
    }
    for (size_t i = 0; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
  }
  // If we have a set of fake indices, but they do not match the number of points in the cloud, update them
  if (fake_indices_ && indices_->size () != (input_->width * input_->height))
  {
    size_t indices_size = indices_->size ();
    indices_->resize (input_->width * input_->height);
    for (size_t i = indices_size; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
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
  indices_.reset (new std::vector<int> (indices->indices));
  fake_indices_ = false;
  use_indices_  = true;
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(PCLBase, PCL_POINT_TYPES)
#endif    // PCL_NO_PRECOMPILE

