/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_

#include <pcl/registration/correspondence_estimation_normal_shooting.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT> void
pcl::registration::CorrespondenceEstimationNormalShooting<PointSource, PointTarget, NormalT>::determineCorrespondences (
    pcl::Correspondences &correspondences, float max_distance)
{
  if (!target_)
  {
    PCL_WARN ("[pcl::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return;
  }

  if (!source_normals_ || !target_normals_)
  {
    PCL_WARN ("[pcl::%s::compute] Datasets containing normals for source/target have not been given!\n", getClassName ().c_str ());
    return;
  }

  if (!initCompute ())
    return;

  typedef typename pcl::traits::fieldList<PointTarget>::type FieldListTarget;
  
  correspondences.resize (indices_->size ());

  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  float min_dist = std::numeric_limits<float>::max ();
  int min_index = 0;
  
  // Set the internal point representation of choice
  if (point_representation_)
    tree_->setPointRepresentation (point_representation_);

  // If the target indices have been given via setIndicesTarget
  if (target_indices_)
    tree_->setInputCloud (target_, target_indices_);
  else
    tree_->setInputCloud (target_);

  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;

  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (isSamePointType<PointSource, PointTarget> ())
  {
    PointTarget pt;
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      if (!tree_->nearestKSearch (input_->points[*idx], k_, nn_indices, nn_dists))
        continue;

      // Among the K nearest neighbours find the one with minimum perpendicular distance to the normal
      min_dist = std::numeric_limits<float>::max ();
      
      for (size_t j = 0; j < nn_indices.size (); j++)
      {
        // computing the distance between a point and a line in 3d. 
        // Reference - http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        pt.x = target_->points[nn_indices[j]].x - input_->points[*idx].x;
        pt.y = target_->points[nn_indices[j]].y - input_->points[*idx].y;
        pt.z = target_->points[nn_indices[j]].z - input_->points[*idx].z;

        NormalT &normal = source_normals_->points[*idx];
        Eigen::Vector3d N (normal.normal_x, normal.normal_y, normal.normal_z);
        Eigen::Vector3d V (pt.x, pt.y, pt.z);
        Eigen::Vector3d C = N.cross (V);
        
        // Check if we have a better correspondence
        float dist = C.dot (C);
        if (dist < min_dist)
        {
          min_dist = dist;
          min_index = j;
        }
      }
      if (min_dist >= max_distance)
        continue;

      corr.index_query = *idx;
      corr.index_match = nn_indices[min_index];
      corr.distance = nn_dists[min_index];//min_dist;
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  else
  {
    PointTarget pt;
    
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      if (!tree_->nearestKSearch (input_->points[*idx], k_, nn_indices, nn_dists))
        continue;

      // Among the K nearest neighbours find the one with minimum perpendicular distance to the normal
      min_dist = std::numeric_limits<float>::max ();
      for (size_t j = 0; j < nn_indices.size (); j++)
      {
        PointSource pt_src;
        // Copy the source data to a target PointTarget format so we can search in the tree
        pcl::for_each_type <FieldListTarget> (pcl::NdConcatenateFunctor <PointSource, PointTarget> (
            input_->points[*idx], 
            pt_src));

        // computing the distance between a point and a line in 3d. 
        // Reference - http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        pt.x = target_->points[nn_indices[j]].x - pt_src.x;
        pt.y = target_->points[nn_indices[j]].y - pt_src.y;
        pt.z = target_->points[nn_indices[j]].z - pt_src.z;
        
        NormalT &normal = source_normals_->points[*idx];
        Eigen::Vector3d N (normal.normal_x, normal.normal_y, normal.normal_z);
        Eigen::Vector3d V (pt.x, pt.y, pt.z);
        Eigen::Vector3d C = N.cross (V);
        
        // Check if we have a better correspondence
        float dist = C.dot (C);
        if (dist < min_dist)
        {
          min_dist = dist;
          min_index = j;
        }
      }
      if (min_dist > max_distance)
        continue;
      corr.index_query = *idx;
      corr.index_match = nn_indices[min_index];
      corr.distance = nn_dists[min_index];//min_dist;
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  correspondences.resize (nr_valid_correspondences);
  deinitCompute ();
}

#endif    // PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_
