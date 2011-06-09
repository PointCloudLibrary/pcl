/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

/**

\author Radu Bogdan Rusu

**/

#include "pcl/pcl_base.h"

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PCLBase<sensor_msgs::PointCloud2>::setInputCloud (const PointCloud2ConstPtr &cloud)
{
  input_ = cloud;

  for (size_t d = 0; d < cloud->fields.size (); ++d)
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
      case sensor_msgs::PointField::INT8:
      case sensor_msgs::PointField::UINT8:
      {
        fsize = 1;
        break;
      }

      case sensor_msgs::PointField::INT16:
      case sensor_msgs::PointField::UINT16:
      {
        fsize = 2;
        break;
      }

      case sensor_msgs::PointField::INT32:
      case sensor_msgs::PointField::UINT32:
      case sensor_msgs::PointField::FLOAT32:
      {
        fsize = 4;
        break;
      }

      case sensor_msgs::PointField::FLOAT64:
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
    field_sizes_[d] = (std::min) (fsize, (int)sizeof (float));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::PCLBase<sensor_msgs::PointCloud2>::deinitCompute ()
{
  // Reset the indices
  if (fake_indices_)
  {
    indices_.reset ();
    fake_indices_ = false;
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::PCLBase<sensor_msgs::PointCloud2>::initCompute ()
{
  // Check if input was set
  if (!input_)
    return (false);

  // If no point indices have been given, construct a set of indices for the entire input point cloud
  if (!indices_)
  {
    fake_indices_ = true;
    std::vector<int> *indices = new std::vector<int> (input_->width * input_->height);
    for (size_t i = 0; i < indices->size (); ++i) { (*indices)[i] = i; }
    indices_.reset (indices);
  }
  return (true);
}

