/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009-2010, Willow Garage, Inc.
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
 * $Id: sac_model_normal_parallel_plane.hpp 34393 2010-11-30 23:02:08Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_

#include "pcl/sample_consensus/sac_model_normal_parallel_plane.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Select all the points which respect the given model coefficients as inliers.
  * \param model_coefficients the coefficients of a plane model that we need to compute distances to
  * \param inliers the resultant model inliers
  * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
  */
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelNormalParallelPlane<PointT, PointNT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers)
{
  if (!normals_)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelNormalParallelPlane::getDistancesToModel] No input dataset containing normals was given!");
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  // Obtain the plane normal
  Eigen::Vector4f coeff = model_coefficients;
  coeff[3] = 0;

  int nr_p = 0;
  inliers.resize (indices_->size ());
  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f p (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);
    double d_euclid = fabs (coeff.dot (p) + model_coefficients[3]);

    // Calculate the angular distance between the point normal and the plane normal
    double d_normal = fabs (getAngle3D (n, coeff));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    if (fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid) < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      nr_p++;
    }
  }
  inliers.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Compute all distances from the cloud data to a given plane model.
  * \param model_coefficients the coefficients of a plane model that we need to compute distances to
  * \param distances the resultant estimated distances
  */
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelNormalParallelPlane<PointT, PointNT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  if (!normals_)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelNormalParallelPlane::getDistancesToModel] No input dataset containing normals was given!");
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  // Obtain the plane normal
  Eigen::Vector4f coeff = model_coefficients;
  coeff[3] = 0;

  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f p (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);
    double d_euclid = fabs (coeff.dot (p) + model_coefficients[3]);

    // Calculate the angular distance between the point normal and the plane normal
    double d_normal = fabs (getAngle3D (n, coeff));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    distances[i] = fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Check whether a model is valid given the user constraints.
  * \param model_coefficients the set of model coefficients
  */
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelNormalParallelPlane<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelNormalParallelPlane::isModelValid] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return (false);
  }

  // Check against template, if given
  if (eps_angle_ > 0.0)
  {
    // Obtain the plane normal
    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;

    Eigen::Vector4f axis (axis_[0], axis_[1], axis_[2], 0);
    double angle_deviation_from_90 = fabs (getAngle3D (axis, coeff));
    angle_deviation_from_90 = fabs (angle_deviation_from_90 - (M_PI/2.0));
    // Check whether the current plane model satisfies our angle threshold criterion with respect to the given axis
    if (angle_deviation_from_90 > eps_angle_)
      return (false);
  }

  if (eps_dist_ > 0.0)
  {
    if (fabs (-model_coefficients[3] - distance_from_origin_) > eps_dist_)
      return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelNormalParallelPlane(PointT, PointNT) template class pcl::SampleConsensusModelNormalParallelPlane<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_


