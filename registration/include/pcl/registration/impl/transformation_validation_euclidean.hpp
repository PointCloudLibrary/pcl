/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_IMPL_H_
#define PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_IMPL_H_

#include <pcl/registration/transformation_validation_euclidean.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> double
pcl::registration::TransformationValidationEuclidean<PointSource, PointTarget>::validateTransformation (
  const PointCloudSourceConstPtr &cloud_src,
  const PointCloudTargetConstPtr &cloud_tgt,
  const Eigen::Matrix4f &transformation_matrix)
{
  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  pcl::PointCloud<PointSource> input_transformed;
  transformPointCloud (*cloud_src, input_transformed, transformation_matrix);

  // Just in case
  if (!tree_)
    tree_.reset (new pcl::KdTreeFLANN<PointTarget>);

  tree_->setInputCloud (cloud_tgt);

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);
    
    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range_)
      continue;

    // Optimization: use getVector4fMap instead, but make sure that the last coordinate is 0!
    Eigen::Vector4f p1 (input_transformed.points[i].x,
                        input_transformed.points[i].y,
                        input_transformed.points[i].z, 0);
    Eigen::Vector4f p2 (cloud_tgt->points[nn_indices[0]].x,
                        cloud_tgt->points[nn_indices[0]].y,
                        cloud_tgt->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm ());
    nr++;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}

#endif /* PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_IMPL_H_ */
