/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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
 * $Id: extract_indices.hpp 1897 2011-07-26 20:35:49Z rusu $
 *
 */

#ifndef PCL_FILTERS_IMPL_CROP_BOX_H_
#define PCL_FILTERS_IMPL_CROP_BOX_H_

#include <pcl/filters/crop_box.h>
#include <pcl/common/eigen.h> // for getTransformation
#include <pcl/common/point_tests.h> // for isFinite
#include <pcl/common/transforms.h> // for transformPoint

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::CropBox<PointT>::applyFilter (Indices &indices)
{
  indices.resize (input_->size ());
  removed_indices_->resize (input_->size ());
  int indices_count = 0;
  int removed_indices_count = 0;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity ();
  Eigen::Affine3f inverse_transform = Eigen::Affine3f::Identity ();

  if (rotation_ != Eigen::Vector3f::Zero ())
  {
    pcl::getTransformation (0, 0, 0,
                            rotation_ (0), rotation_ (1), rotation_ (2),
                            transform);
    inverse_transform = transform.inverse ();
  }

  bool transform_matrix_is_identity = transform_.matrix ().isIdentity ();
  bool translation_is_zero = (translation_ == Eigen::Vector3f::Zero ());
  bool inverse_transform_matrix_is_identity = inverse_transform.matrix ().isIdentity ();

  for (const auto index : *indices_)
  {
    if (!input_->is_dense)
      // Check if the point is invalid
      if (!isFinite ((*input_)[index]))
        continue;

    // Get local point
    PointT local_pt = (*input_)[index];

    // Transform point to world space
    if (!transform_matrix_is_identity)
      local_pt = pcl::transformPoint<PointT> (local_pt, transform_);

    if (!translation_is_zero)
    {
      local_pt.x -= translation_ (0);
      local_pt.y -= translation_ (1);
      local_pt.z -= translation_ (2);
    }

    // Transform point to local space of crop box
    if (!inverse_transform_matrix_is_identity)
      local_pt = pcl::transformPoint<PointT> (local_pt, inverse_transform);

    // If outside the cropbox
    if ( (local_pt.x < min_pt_[0] || local_pt.y < min_pt_[1] || local_pt.z < min_pt_[2]) ||
         (local_pt.x > max_pt_[0] || local_pt.y > max_pt_[1] || local_pt.z > max_pt_[2]))
    {
      if (negative_)
        indices[indices_count++] = index;
      else if (extract_removed_indices_)
        (*removed_indices_)[removed_indices_count++] = index;
    }
    // If inside the cropbox
    else
    {
      if (negative_ && extract_removed_indices_)
        (*removed_indices_)[removed_indices_count++] = index;
      else if (!negative_) 
        indices[indices_count++] = index;
    }
  }
  indices.resize (indices_count);
  removed_indices_->resize (removed_indices_count);
}

#define PCL_INSTANTIATE_CropBox(T) template class PCL_EXPORTS pcl::CropBox<T>;

#endif    // PCL_FILTERS_IMPL_CROP_BOX_H_
