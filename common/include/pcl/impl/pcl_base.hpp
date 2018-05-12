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
#ifndef PCL_PCL_IMPL_BASE_HPP_
#define PCL_PCL_IMPL_BASE_HPP_

#include <pcl/pcl_base.h>
#include <pcl/console/print.h>
#include <cstddef>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::PCLBase<PointT>::PCLBase ()
  : input_ ()
  , indices_ ()
  , use_indices_ (false)
  , fake_indices_ (false)
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::PCLBase<PointT>::PCLBase (const PCLBase& base)
  : input_ (base.input_)
  , indices_ (base.indices_)
  , use_indices_ (base.use_indices_)
  , fake_indices_ (base.fake_indices_)
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PCLBase<PointT>::setInputCloud (const PointCloudConstPtr &cloud)
{ 
  input_ = cloud; 
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PCLBase<PointT>::setIndices (const IndicesPtr &indices)
{
  indices_ = indices;
  fake_indices_ = false;
  use_indices_  = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PCLBase<PointT>::setIndices (const IndicesConstPtr &indices)
{
  indices_.reset (new std::vector<int> (*indices));
  fake_indices_ = false;
  use_indices_  = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PCLBase<PointT>::setIndices (const PointIndicesConstPtr &indices)
{
  indices_.reset (new std::vector<int> (indices->indices));
  fake_indices_ = false;
  use_indices_  = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::PCLBase<PointT>::setIndices (size_t row_start, size_t col_start, size_t nb_rows, size_t nb_cols)
{
  if ((nb_rows > input_->height) || (row_start > input_->height))
  {
    PCL_ERROR ("[PCLBase::setIndices] cloud is only %d height", input_->height);
    return;
  }

  if ((nb_cols > input_->width) || (col_start > input_->width))
  {
    PCL_ERROR ("[PCLBase::setIndices] cloud is only %d width", input_->width);
    return;
  }

  size_t row_end = row_start + nb_rows;
  if (row_end > input_->height)
  {
    PCL_ERROR ("[PCLBase::setIndices] %d is out of rows range %d", row_end, input_->height);
    return;
  }

  size_t col_end = col_start + nb_cols;
  if (col_end > input_->width)
  {
    PCL_ERROR ("[PCLBase::setIndices] %d is out of columns range %d", col_end, input_->width);
    return;
  }

  indices_.reset (new std::vector<int>);
  indices_->reserve (nb_cols * nb_rows);
  for(size_t i = row_start; i < row_end; i++)
    for(size_t j = col_start; j < col_end; j++)
      indices_->push_back (static_cast<int> ((i * input_->width) + j));
  fake_indices_ = false;
  use_indices_  = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PCLBase<PointT>::initCompute ()
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
      indices_->resize (input_->points.size ());
    }
    catch (const std::bad_alloc&)
    {
      PCL_ERROR ("[initCompute] Failed to allocate %lu indices.\n", input_->points.size ());
    }
    for (size_t i = 0; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
  }

  // If we have a set of fake indices, but they do not match the number of points in the cloud, update them
  if (fake_indices_ && indices_->size () != input_->points.size ())
  {
    size_t indices_size = indices_->size ();
    indices_->resize (input_->points.size ());
    for (size_t i = indices_size; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PCLBase<PointT>::deinitCompute ()
{
  return (true);
}

#define PCL_INSTANTIATE_PCLBase(T) template class PCL_EXPORTS pcl::PCLBase<T>;

#endif  //#ifndef PCL_PCL_IMPL_BASE_HPP_

