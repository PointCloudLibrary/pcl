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
pcl::SampleConsensusModelSphere<PointT>::isSampleGood (const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelSphere<PointT>::computeModelCoefficients (
      const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  // Need 4 samples
  if (samples.size () != sample_size_)
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
  {
    return (false);             // the points don't define a sphere!
  }

  for (int i = 0; i < 4; ++i)
  {
    temp (i, 0) = (input_->points[samples[i]].x) * (input_->points[samples[i]].x) +
                  (input_->points[samples[i]].y) * (input_->points[samples[i]].y) +
                  (input_->points[samples[i]].z) * (input_->points[samples[i]].z);
  }
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
  model_coefficients.resize (model_size_);
  model_coefficients[0] = 0.5f * m12 / m11;
  model_coefficients[1] = 0.5f * m13 / m11;
  model_coefficients[2] = 0.5f * m14 / m11;
  // Radius
  model_coefficients[3] = std::sqrt (model_coefficients[0] * model_coefficients[0] +
                                     model_coefficients[1] * model_coefficients[1] +
                                     model_coefficients[2] * model_coefficients[2] - m15 / m11);

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius
    distances[i] = std::abs (std::sqrt (
                               ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                               ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                               ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                               ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) +

                               ( input_->points[(*indices_)[i]].z - model_coefficients[2] ) *
                               ( input_->points[(*indices_)[i]].z - model_coefficients[2] )
                               ) - model_coefficients[3]);
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  inliers.clear ();
  error_sqr_dists_.clear ();
  inliers.reserve (indices_->size ());
  error_sqr_dists_.reserve (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    double distance = std::abs (std::sqrt (
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
      inliers.push_back ((*indices_)[i]);
      error_sqr_dists_.push_back (static_cast<double> (distance));
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelSphere<PointT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

  std::size_t nr_p = 0;

  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the sphere as the difference between
    // dist(point,sphere_origin) and sphere_radius
    if (std::abs (std::sqrt (
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
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] Given model is invalid!\n");
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size () <= sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] Not enough inliers to refine/optimize the model's coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
    return;
  }

  OptimizationFunctor functor (this, inliers);
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
      const Indices &, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool) const
{
  // Needs a valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::projectPoints] Given model is invalid!\n");
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
      const std::set<index_t> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelSphere::doSamplesVerifyModel] Given model is invalid!\n");
    return (false);
  }

  for (const auto &index : indices)
  {
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius
    if (std::abs (sqrt (
                    ( input_->points[index].x - model_coefficients[0] ) *
                    ( input_->points[index].x - model_coefficients[0] ) +
                    ( input_->points[index].y - model_coefficients[1] ) *
                    ( input_->points[index].y - model_coefficients[1] ) +
                    ( input_->points[index].z - model_coefficients[2] ) *
                    ( input_->points[index].z - model_coefficients[2] )
                   ) - model_coefficients[3]) > threshold)
    {
      return (false);
    }
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelSphere(T) template class PCL_EXPORTS pcl::SampleConsensusModelSphere<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_SPHERE_H_

