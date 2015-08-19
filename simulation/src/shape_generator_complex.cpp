/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2014-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 */

#include <pcl/simulation/shape_generator_complex.h>
typedef pcl::simulation::GeometricShapeBase::PointCloudT PointCloudT;

// ------------------ Multishape definitions --------------------------------

bool
pcl::simulation::MultiShape::isInside (const PointT &point) const
{
  // since the is_inside methods for all shapes expects a point in its respective reference frame, we have to manually transform the point into the shape reference frames
  for (std::size_t cs = 0; cs < shapes_ptrs_.size (); ++cs)
  {
    PointT Backtransformed_Point = pcl::transformPoint (point, shapes_ptrs_[cs]->effective_transform_.inverse ());
    if (shapes_ptrs_[cs]->isInside (Backtransformed_Point))
      return (true);
  }
  return (false);
}

PointCloudT::Ptr
pcl::simulation::MultiShape::generate (float resolution)
{
  PointCloudT::Ptr cloud_ptr (new PointCloudT ());
  std::vector <PointCloudT::Ptr> part_clouds;
  part_clouds.resize (shapes_ptrs_.size ());
  for (std::size_t cs = 0; cs < shapes_ptrs_.size (); ++cs)
  {
    part_clouds[cs] = shapes_ptrs_[cs]->generate (resolution);
  }
  
// if delete overlap is true, than compare each cloud to each other cloud and delete the points within the other cloud
  if (delete_overlap_)
    for (std::size_t cs1 = 0; cs1 < shapes_ptrs_.size (); ++cs1)
    {
      for (std::size_t cs2 = 0; cs2 < shapes_ptrs_.size (); ++cs2)
      {
        if (cs1 != cs2)
        {
          shapes_ptrs_[cs1]->deletePointsFromOtherCloud (part_clouds[cs2]);
        }
      }
    }

  // if we want to keep labels of parts separate we need to define a mapping from part_id, label_id -> new_label_id
  unsigned int current_max_label = 0;
  // add points to the shape using the actual label handling policy
  for (std::size_t cs = 0; cs < part_clouds.size (); ++cs)
  {
    PointCloudT::Ptr part_cloud_ptr = part_clouds[cs];
    std::map <unsigned int, unsigned int> part_label_to_object_label_map;
    for (PointCloudT::iterator itr = part_cloud_ptr->begin (); itr < part_cloud_ptr->end (); ++itr)
    {
      switch (label_handling_)
      {
        case Single:
          itr->label = 0;
          break;
        case Merge:
          itr->label = cs;
          break;
        case Append:
          if (part_label_to_object_label_map.find (itr->label) == part_label_to_object_label_map.end ())
          {
            part_label_to_object_label_map[itr->label] = current_max_label;
            ++current_max_label;
          }
          itr->label = part_label_to_object_label_map[itr->label];
          break;
        case Preserve:
          // we do not need to change the label here, since we will preserve the id which has been assigned to the part already.
          break;
      }
      cloud_ptr->push_back (*itr);
    }
  }
  
  applyTransformations (cloud_ptr);
  return (cloud_ptr);
}

// ------------------ Cut shape definitions --------------------------------

PointCloudT::Ptr
pcl::simulation::CutShape::generate (float resolution)
{
  PointCloudT::Ptr cloud_ptr (new PointCloudT ());
  PointCloudT::Ptr outer_cloud_ptr = outer_shape_ptr_->generate (resolution);
  PointCloudT::Ptr inner_cloud_ptr = inner_shape_ptr_->generate (resolution); 

  // delete the inliers of the outer_shape.
  inner_shape_ptr_->deletePointsFromOtherCloud (outer_cloud_ptr, true);

  // delete the outliers from the inner shape.
  outer_shape_ptr_->deletePointsFromOtherCloud (inner_cloud_ptr, false);

  // flip normals of the inner shape.
  flipNormals (inner_cloud_ptr);

  // combine both clouds.
  *cloud_ptr = *outer_cloud_ptr + *inner_cloud_ptr;

  // apply transformation which may have been set before generate was called
  applyTransformations (cloud_ptr);
  return (cloud_ptr);
}

bool
pcl::simulation::CutShape::isInside (const PointT &point) const
{
  // since the is_inside for _OuterShape and _InsideShape expects a point in its own reference frame, we have to manually transform the point into the Outside and Inside shape reference frame
  PointT outside_backtransformed_point, inside_backtransformed_point;
  outside_backtransformed_point = pcl::transformPoint (point, outer_shape_ptr_->effective_transform_.inverse ());
  inside_backtransformed_point = pcl::transformPoint (point, inner_shape_ptr_->effective_transform_.inverse ());

  return (outer_shape_ptr_->isInside (outside_backtransformed_point) && !inner_shape_ptr_->isInside (inside_backtransformed_point));
}
