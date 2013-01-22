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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_IMPL_H_
#define PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_IMPL_H_

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> double
pcl::registration::TransformationValidationEuclidean<PointSource, PointTarget, Scalar>::validateTransformation (
  const PointCloudSourceConstPtr &cloud_src,
  const PointCloudTargetConstPtr &cloud_tgt,
  const Matrix4 &transformation_matrix) const
{
  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  pcl::PointCloud<PointSource> input_transformed;
  //transformPointCloud (*cloud_src, input_transformed, transformation_matrix);
  input_transformed.resize (cloud_src->size ());
  for (size_t i = 0; i < cloud_src->size (); ++i)
  {
    const PointSource &src = cloud_src->points[i];
    PointTarget &tgt = input_transformed.points[i];
    tgt.x = static_cast<float> (transformation_matrix (0, 0) * src.x + transformation_matrix (0, 1) * src.y + transformation_matrix (0, 2) * src.z + transformation_matrix (0, 3));
    tgt.y = static_cast<float> (transformation_matrix (1, 0) * src.x + transformation_matrix (1, 1) * src.y + transformation_matrix (1, 2) * src.z + transformation_matrix (1, 3));
    tgt.z = static_cast<float> (transformation_matrix (2, 0) * src.x + transformation_matrix (2, 1) * src.y + transformation_matrix (2, 2) * src.z + transformation_matrix (2, 3));
   }

  typename MyPointRepresentation::ConstPtr point_rep (new MyPointRepresentation);
  if (!force_no_recompute_)
  {
    tree_->setPointRepresentation (point_rep);
    tree_->setInputCloud (cloud_tgt);
  }

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

    // Calculate the fitness score
    fitness_score += nn_dists[0];
    ++nr;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}

#endif    // PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_IMPL_H_

