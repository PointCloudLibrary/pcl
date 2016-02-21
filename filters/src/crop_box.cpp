/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2015, Google, Inc.
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

#include <pcl/filters/impl/crop_box.hpp>

///////////////////////////////////////////////////////////////////////////////
void
pcl::CropBox<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
{
  // Resize output cloud to sample size
  output.data.resize (input_->data.size ());
  removed_indices_->resize (input_->data.size ());

  // Copy the common fields
  output.fields = input_->fields;
  output.is_bigendian = input_->is_bigendian;
  output.row_step = input_->row_step;
  output.point_step = input_->point_step;
  output.height = 1;

  int indices_count = 0;
  int removed_indices_count = 0;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f inverse_transform = Eigen::Affine3f::Identity();

  if (rotation_ != Eigen::Vector3f::Zero ())
  {
    pcl::getTransformation (0, 0, 0,
                            rotation_ (0), rotation_ (1), rotation_ (2),
                            transform);
    inverse_transform = transform.inverse();
  }

  //PointXYZ local_pt;
  Eigen::Vector3f local_pt (Eigen::Vector3f::Zero ());

  bool transform_matrix_is_identity = transform_.matrix ().isIdentity ();
  bool translation_is_zero = (translation_ != Eigen::Vector3f::Zero ());
  bool inverse_transform_matrix_is_identity = inverse_transform.matrix ().isIdentity ();

  for (size_t index = 0; index < indices_->size (); ++index)
  {
    // Get local point
    int point_offset = ((*indices_)[index] * input_->point_step);
    int offset = point_offset + input_->fields[x_idx_].offset;
    memcpy (&local_pt, &input_->data[offset], sizeof (float)*3);

    // Check if the point is invalid
    if (!pcl_isfinite (local_pt.x ()) ||
        !pcl_isfinite (local_pt.y ()) ||
        !pcl_isfinite (local_pt.z ()))
      continue;

    // Transform point to world space
    if (!transform_matrix_is_identity)
      local_pt = transform_ * local_pt;

    if (translation_is_zero)
    {
      local_pt.x () = local_pt.x () - translation_ (0);
      local_pt.y () = local_pt.y () - translation_ (1);
      local_pt.z () = local_pt.z () - translation_ (2);
    }

    // Transform point to local space of crop box
    if (!inverse_transform_matrix_is_identity)
      local_pt = inverse_transform * local_pt;

    // If outside the cropbox
    if ( (local_pt.x () < min_pt_[0] || local_pt.y () < min_pt_[1] || local_pt.z () < min_pt_[2]) ||
         (local_pt.x () > max_pt_[0] || local_pt.y () > max_pt_[1] || local_pt.z () > max_pt_[2]))
    {
      if (negative_)
      {
        memcpy (&output.data[indices_count++ * output.point_step],
                &input_->data[index * output.point_step], output.point_step);
      }
      else if (extract_removed_indices_)
      {
        (*removed_indices_)[removed_indices_count++] = static_cast<int> (index);
      }
    }
    // If inside the cropbox
    else
    {
      if (negative_ && extract_removed_indices_)
      {
        (*removed_indices_)[removed_indices_count++] = static_cast<int> (index);
      }
      else if (!negative_) {
        memcpy (&output.data[indices_count++ * output.point_step],
                &input_->data[index * output.point_step], output.point_step);
      }
    }
  }
  output.width = indices_count;
  output.row_step = output.point_step * output.width;
  output.data.resize (output.width * output.height * output.point_step);

  removed_indices_->resize (removed_indices_count);
}

///////////////////////////////////////////////////////////////////////////////
void
pcl::CropBox<pcl::PCLPointCloud2>::applyFilter (std::vector<int> &indices)
{
  indices.resize (input_->width * input_->height);
  removed_indices_->resize (input_->width * input_->height);

  int indices_count = 0;
  int removed_indices_count = 0;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Affine3f inverse_transform = Eigen::Affine3f::Identity();

  if (rotation_ != Eigen::Vector3f::Zero ())
  {
    pcl::getTransformation (0, 0, 0,
                            rotation_ (0), rotation_ (1), rotation_ (2),
                            transform);
    inverse_transform = transform.inverse();
  }

  //PointXYZ local_pt;
  Eigen::Vector3f local_pt (Eigen::Vector3f::Zero ());

  bool transform_matrix_is_identity = transform_.matrix ().isIdentity ();
  bool translation_is_zero = (translation_ != Eigen::Vector3f::Zero ());
  bool inverse_transform_matrix_is_identity = inverse_transform.matrix ().isIdentity ();

  for (size_t index = 0; index < indices_->size (); index++)
  {
    // Get local point
    int point_offset = ((*indices_)[index] * input_->point_step);
    int offset = point_offset + input_->fields[x_idx_].offset;
    memcpy (&local_pt, &input_->data[offset], sizeof (float)*3);

    // Transform point to world space
    if (!transform_matrix_is_identity)
      local_pt = transform_ * local_pt;

    if (translation_is_zero)
    {
      local_pt.x () -= translation_ (0);
      local_pt.y () -= translation_ (1);
      local_pt.z () -= translation_ (2);
    }

    // Transform point to local space of crop box
    if (!(inverse_transform_matrix_is_identity))
      local_pt = inverse_transform * local_pt;

    // If outside the cropbox
    if ( (local_pt.x () < min_pt_[0] || local_pt.y () < min_pt_[1] || local_pt.z () < min_pt_[2]) ||
         (local_pt.x () > max_pt_[0] || local_pt.y () > max_pt_[1] || local_pt.z () > max_pt_[2]))
    {
      if (negative_)
      {
        indices[indices_count++] = (*indices_)[index];
      }
      else if (extract_removed_indices_)
      {
        (*removed_indices_)[removed_indices_count++] = static_cast<int> (index);
      }
    }
    // If inside the cropbox
    else
    {
      if (negative_ && extract_removed_indices_)
      {
        (*removed_indices_)[removed_indices_count++] = static_cast<int> (index);
      }
      else if (!negative_) {
        indices[indices_count++] = (*indices_)[index];
      }
    }
  }

  indices.resize (indices_count);
  removed_indices_->resize (removed_indices_count);
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

PCL_INSTANTIATE(CropBox, PCL_XYZ_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE

