/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_SPHERE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_SPHERE_H_

#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelSphere<PointT>::isSampleGood (const std::vector<int> &) const
{
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelSphere<PointT>::computeModelCoefficients (
      const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 4 samples
  if (samples.size () != 4)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  Eigen::Matrix4f temp;
  for (int i = 0; i < 4; i++)
  {
    temp (i, 0) = input_->points[samples[i]].x;
    temp (i, 1) = input_->points[samples[i]].y;
    temp (i, 2) = input_->points[samples[i]].z;
    temp (i, 3) = 1;
  }
  float m11 = temp.determinant ();
  if (m11 == 0)
    return (false);             // the points don't define a sphere!

  for (int i = 0; i < 4; ++i)
    temp (i, 0) = (input_->points[samples[i]].x) * (input_->points[samples[i]].x) +
                  (input_->points[samples[i]].y) * (input_->points[samples[i]].y) +
                  (input_->points[samples[i]].z) * (input_->points[samples[i]].z);
  float m12 = temp.determinant ();

  for (int i = 0; i < 4; ++i)
  {
    temp (i, 1) = temp (i, 0);
    temp (i, 0) = input_->points[samples[i]].x;
  }
  float m13 = temp.determinant ();

  for (int i = 0; i < 4; ++i)
  {
    temp (i, 2) = temp (i, 1);
    temp (i, 1) = input_->points[samples[i]].y;
  }
  float m14 = temp.determinant ();

  for (int i = 0; i < 4; ++i)
  {
    temp (i, 0) = temp (i, 2);
    temp (i, 1) = input_->points[samples[i]].x;
    temp (i, 2) = input_->points[samples[i]].y;
    temp (i, 3) = input_->points[samples[i]].z;
  }
  float m15 = temp.determinant ();

  // Center (x , y, z)
  model_coefficients.resize (4);
  model_coefficients[0] = 0.5f * m12 / m11;
  model_coefficients[1] = 0.5f * m13 / m11;
  model_coefficients[2] = 0.5f * m14 / m11;
  // Radius
  model_coefficients[3] = sqrtf (
                                 model_coefficients[0] * model_coefficients[0] +
                                 model_coefficients[1] * model_coefficients[1] +
                                 model_coefficients[2] * model_coefficients[2] - m15 / m11);

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius
    distances[i] = fabs (sqrtf (
                               ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                               ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                               ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                               ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) +

                               ( input_->points[(*indices_)[i]].z - model_coefficients[2] ) *
                               ( input_->points[(*indices_)[i]].z - model_coefficients[2] )
                               ) - model_coefficients[3]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  int nr_p = 0;
  inliers.resize (indices_->size ());
  error_sqr_dists_.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    double distance = fabs (sqrtf (
                          ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                          ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                          ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                          ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) +

                          ( input_->points[(*indices_)[i]].z - model_coefficients[2] ) *
                          ( input_->points[(*indices_)[i]].z - model_coefficients[2] )
                          ) - model_coefficients[3]);
    // Calculate the distance from the point to the sphere as the difference between
    // dist(point,sphere_origin) and sphere_radius
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

//////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::SampleConsensusModelSphere<PointT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

  int nr_p = 0;

  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the sphere as the difference between
    // dist(point,sphere_origin) and sphere_radius
    if (fabs (sqrtf (
                    ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                    ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                    ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                    ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) +

                    ( input_->points[(*indices_)[i]].z - model_coefficients[2] ) *
                    ( input_->points[(*indices_)[i]].z - model_coefficients[2] )
                    ) - model_coefficients[3]) < threshold)
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (model_coefficients.size () != 4)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return;
  }

  // Need at least 4 samples
  if (inliers.size () <= 4)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] Not enough inliers found to support a model (%lu)! Returning the same coefficients.\n", inliers.size ());
    return;
  }

  tmp_inliers_ = &inliers;

  OptimizationFunctor functor (static_cast<int> (inliers.size ()), this);
  Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm (num_diff);
  int info = lm.minimize (optimized_coefficients);

  // Compute the L2 norm of the residuals
  PCL_DEBUG ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g \nFinal solution: %g %g %g %g\n",
             info, lm.fvec.norm (), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3], optimized_coefficients[0], optimized_coefficients[1], optimized_coefficients[2], optimized_coefficients[3]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::projectPoints (
      const std::vector<int> &, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::projectPoints] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return;
  }

  // Allocate enough space and copy the basics
  projected_points.points.resize (input_->points.size ());
  projected_points.header   = input_->header;
  projected_points.width    = input_->width;
  projected_points.height   = input_->height;
  projected_points.is_dense = input_->is_dense;

  PCL_WARN ("[pcl::SampleConsensusModelSphere::projectPoints] Not implemented yet.\n");
  projected_points.points = input_->points;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelSphere<PointT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, const double threshold)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::doSamplesVerifyModel] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }

  for (std::set<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius
    if (fabs (sqrt (
                    ( input_->points[*it].x - model_coefficients[0] ) *
                    ( input_->points[*it].x - model_coefficients[0] ) +
                    ( input_->points[*it].y - model_coefficients[1] ) *
                    ( input_->points[*it].y - model_coefficients[1] ) +
                    ( input_->points[*it].z - model_coefficients[2] ) *
                    ( input_->points[*it].z - model_coefficients[2] )
                   ) - model_coefficients[3]) > threshold)
      return (false);

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelSphere(T) template class PCL_EXPORTS pcl::SampleConsensusModelSphere<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_SPHERE_H_

