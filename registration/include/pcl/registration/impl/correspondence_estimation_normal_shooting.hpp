/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_

#include <pcl/registration/correspondence_estimation_normal_shooting.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename NormalT> void
pcl::registration::CorrespondenceEstimationNormalShooting<PointSource, PointTarget, NormalT>::determineCorrespondences (
    pcl::Correspondences &correspondences, float max_distance)
{
  if (!initCompute ())
    return;

  if (!target_)
  {
    PCL_WARN ("[pcl::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return;
  }
  correspondences.resize (indices_->size ());

  float min_dist = std::numeric_limits<float>::max ();
  float dist = 0.0;
  int min_index = 0;
  tree_->setInputCloud (target_);

  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  pcl::Correspondence corr;

  for (size_t i = 0; i < source_normals_->points.size (); i++)
  {
    float *normal = source_normals_->points[i].normal;

    tree_->nearestKSearch (input_->points[i], k_, nn_indices, nn_dists);

    // Among the K nearest neighbours find the one with minimum perpendicular distance to the normal
    min_dist = std::numeric_limits<float>::max ();
    for (size_t j = 0; j < nn_indices.size (); j++)
    {
      int q = nn_indices[j];
      // computing the distance between a point and a line in 3d. 
      // Reference - http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
      PointTarget pt;
      pt.x = target_->points[q].x - input_->points[i].x;
      pt.y = target_->points[q].y - input_->points[i].y;
      pt.z = target_->points[q].z - input_->points[i].z;
      Eigen::Vector3d N (normal[0], normal[1], normal[2]);
      Eigen::Vector3d V (pt.x, pt.y, pt.z);
      Eigen::Vector3d C = N.cross (V);
      dist = C.dot (C);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_index = q;
      }
    }
    if (min_dist > max_distance)
      continue;
    corr.index_query = i;
    corr.index_match = min_index;
    corr.distance = min_dist;
    correspondences[i] = corr;
  }
  deinitCompute ();
}

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_ */
