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
  Indices indices;
  if (keep_organized_)
  {
    bool temp = extract_removed_indices_;
    extract_removed_indices_ = true;
    applyFilter (indices);
    extract_removed_indices_ = temp;
    PCL_DEBUG ("[pcl::%s<pcl::PCLPointCloud2>::applyFilter] Removing %lu points of %lu points.\n",
               filter_name_.c_str (), removed_indices_->size(), input_->height * input_->width);

    output = *input_;

    // Get x, y, z fields. We should not just assume that they are the first fields of each point
    std::vector<std::uint32_t> offsets;
    for (const pcl::PCLPointField &field : input_->fields)
    {
      if (field.name == "x" ||
          field.name == "y" ||
          field.name == "z")
        offsets.push_back (field.offset);
    }
    PCL_DEBUG ("[pcl::%s<pcl::PCLPointCloud2>::applyFilter] Found %lu fields called 'x', 'y', or 'z'.\n",
               filter_name_.c_str (), offsets.size());

    // For every "removed" point, set the x, y, z fields to user_filter_value_
    const static float user_filter_value = user_filter_value_;
    for (const auto ri : *removed_indices_) // ri = removed index
    {
      auto* pt_data = reinterpret_cast<std::uint8_t*> (&output.data[ri * output.point_step]);
      for (const auto &offset : offsets)
      {
        memcpy (pt_data + offset, &user_filter_value, sizeof (float));
      }
    }
    if (!std::isfinite (user_filter_value_))
    {
      PCL_DEBUG ("[pcl::%s<pcl::PCLPointCloud2>::applyFilter] user_filter_value_ is %f, which is not finite, "
                 "so the is_dense field of the output will be set to false.\n", filter_name_.c_str (), user_filter_value_);
      output.is_dense = false;
    }
  }
  else
  {
    // Here indices is used, not removed_indices_, so no need to change extract_removed_indices_.
    applyFilter (indices);
    PCL_DEBUG ("[pcl::%s<pcl::PCLPointCloud2>::applyFilter] Removing %lu points of %lu points.\n",
               filter_name_.c_str (), (input_->height * input_->width) - indices.size(), input_->height * input_->width);
    pcl::copyPointCloud (*input_, indices, output);
  }
}

///////////////////////////////////////////////////////////////////////////////
void
pcl::CropBox<pcl::PCLPointCloud2>::applyFilter (Indices &indices)
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
  bool translation_is_not_zero = (translation_ != Eigen::Vector3f::Zero ());
  bool inverse_transform_matrix_is_identity = inverse_transform.matrix ().isIdentity ();

  for (const auto index : *indices_)
  {
    // Get local point
    std::size_t point_offset = static_cast<std::size_t>(index) * input_->point_step;
    std::size_t offset = point_offset + input_->fields[x_idx_].offset;
    memcpy (local_pt.data (), &input_->data[offset], sizeof (float)*3);

    // Transform point to world space
    if (!transform_matrix_is_identity)
      local_pt = transform_ * local_pt;

    if (translation_is_not_zero)
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
        indices[indices_count++] = index;
      }
      else if (extract_removed_indices_)
      {
        (*removed_indices_)[removed_indices_count++] = index;
      }
    }
    // If inside the cropbox
    else
    {
      if (negative_ && extract_removed_indices_)
      {
        (*removed_indices_)[removed_indices_count++] = index;
      }
      else if (!negative_) {
        indices[indices_count++] = index;
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

