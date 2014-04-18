/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id: sac_model_normal_sphere.hpp schrandt $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_SPHERE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_SPHERE_H_

#include <pcl/sample_consensus/sac_model_normal_sphere.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelNormalSphere<PointT, PointNT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
{
  if (!normals_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelNormalSphere::selectWithinDistance] No input dataset containing normals was given!\n");
    inliers.clear ();
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  // Obtain the sphere center
  Eigen::Vector4f center = model_coefficients;
  center[3] = 0;

  int nr_p = 0;
  inliers.resize (indices_->size ());
  error_sqr_dists_.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the sphere center as the difference between
    // dist(point,sphere_origin) and sphere_radius
    Eigen::Vector4f p (input_->points[(*indices_)[i]].x, 
                       input_->points[(*indices_)[i]].y,
                       input_->points[(*indices_)[i]].z, 
                       0);

    Eigen::Vector4f n (normals_->points[(*indices_)[i]].normal[0], 
                       normals_->points[(*indices_)[i]].normal[1], 
                       normals_->points[(*indices_)[i]].normal[2], 
                       0);

    Eigen::Vector4f n_dir = p - center;
    double d_euclid = fabs (n_dir.norm () - model_coefficients[3]);

    // Calculate the angular distance between the point normal and the plane normal
    double d_normal = fabs (getAngle3D (n, n_dir));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    double distance = fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid); 
    if (distance < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      error_sqr_dists_[nr_p] = static_cast<double> (distance);
      ++nr_p;
    }
  }
  inliers.resize (nr_p);
  error_sqr_dists_.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> int
pcl::SampleConsensusModelNormalSphere<PointT, PointNT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients,  const double threshold)
{
  if (!normals_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelNormalSphere::getDistancesToModel] No input dataset containing normals was given!\n");
    return (0);
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return(0);


  // Obtain the shpere centroid
  Eigen::Vector4f center = model_coefficients;
  center[3] = 0;

  int nr_p = 0;

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the sphere centroid as the difference between
    // dist(point,sphere_origin) and sphere_radius
    Eigen::Vector4f p (input_->points[(*indices_)[i]].x, 
                       input_->points[(*indices_)[i]].y, 
                       input_->points[(*indices_)[i]].z, 
                       0);

    Eigen::Vector4f n (normals_->points[(*indices_)[i]].normal[0], 
                       normals_->points[(*indices_)[i]].normal[1], 
                       normals_->points[(*indices_)[i]].normal[2], 
                       0);

    Eigen::Vector4f n_dir = (p-center);
    double d_euclid = fabs (n_dir.norm () - model_coefficients[3]);
    //
    // Calculate the angular distance between the point normal and the plane normal
    double d_normal = fabs (getAngle3D (n, n_dir));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    if (fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid) < threshold)
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelNormalSphere<PointT, PointNT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  if (!normals_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelNormalSphere::getDistancesToModel] No input dataset containing normals was given!\n");
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  // Obtain the sphere centroid
  Eigen::Vector4f center = model_coefficients;
  center[3] = 0;

  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the sphere as the difference between
    // dist(point,sphere_origin) and sphere_radius
    Eigen::Vector4f p (input_->points[(*indices_)[i]].x, 
                       input_->points[(*indices_)[i]].y, 
                       input_->points[(*indices_)[i]].z, 
                       0);

    Eigen::Vector4f n (normals_->points[(*indices_)[i]].normal[0], 
                       normals_->points[(*indices_)[i]].normal[1], 
                       normals_->points[(*indices_)[i]].normal[2], 
                       0);

    Eigen::Vector4f n_dir = (p-center);
    double d_euclid = fabs (n_dir.norm () - model_coefficients[3]);
    //
    // Calculate the angular distance between the point normal and the plane normal
    double d_normal = fabs (getAngle3D (n, n_dir));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    distances[i] = fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool 
pcl::SampleConsensusModelNormalSphere<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelNormalSphere::selectWithinDistance] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }

  if (radius_min_ != -std::numeric_limits<double>::max() && model_coefficients[3] < radius_min_)
    return (false);
  if (radius_max_ != std::numeric_limits<double>::max() && model_coefficients[3] > radius_max_)
    return (false);

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelNormalSphere(PointT, PointNT) template class PCL_EXPORTS pcl::SampleConsensusModelNormalSphere<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_SPHERE_H_

