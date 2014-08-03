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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_H_

#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle2D<PointT>::isSampleGood(const std::vector<int> &samples) const
{
  // Get the values at the two points
  Eigen::Array2d p0 (input_->points[samples[0]].x, input_->points[samples[0]].y);
  Eigen::Array2d p1 (input_->points[samples[1]].x, input_->points[samples[1]].y);
  Eigen::Array2d p2 (input_->points[samples[2]].x, input_->points[samples[2]].y);

  // Compute the segment values (in 2d) between p1 and p0
  p1 -= p0;
  // Compute the segment values (in 2d) between p2 and p0
  p2 -= p0;

  Eigen::Array2d dy1dy2 = p1 / p2;

  return (dy1dy2[0] != dy1dy2[1]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle2D<PointT>::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 3 samples
  if (samples.size () != 3)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  model_coefficients.resize (3);

  Eigen::Vector2d p0 (input_->points[samples[0]].x, input_->points[samples[0]].y);
  Eigen::Vector2d p1 (input_->points[samples[1]].x, input_->points[samples[1]].y);
  Eigen::Vector2d p2 (input_->points[samples[2]].x, input_->points[samples[2]].y);

  Eigen::Vector2d u = (p0 + p1) / 2.0;
  Eigen::Vector2d v = (p1 + p2) / 2.0;

  Eigen::Vector2d p1p0dif = p1 - p0;
  Eigen::Vector2d p2p1dif = p2 - p1;
  Eigen::Vector2d uvdif   = u - v;

  Eigen::Vector2d m (- p1p0dif[0] / p1p0dif[1], - p2p1dif[0] / p2p1dif[1]);

  // Center (x, y)
  model_coefficients[0] = static_cast<float> ((m[0] * u[0] -  m[1] * v[0]  - uvdif[1] )             / (m[0] - m[1]));
  model_coefficients[1] = static_cast<float> ((m[0] * m[1] * uvdif[0] +  m[0] * v[1] - m[1] * u[1]) / (m[0] - m[1]));

  // Radius
  model_coefficients[2] = static_cast<float> (sqrt ((model_coefficients[0] - p0[0]) * (model_coefficients[0] - p0[0]) +
                                                    (model_coefficients[1] - p0[1]) * (model_coefficients[1] - p0[1])));
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle2D<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
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
    // Calculate the distance from the point to the circle as the difference between
    // dist(point,circle_origin) and circle_radius
    distances[i] = fabsf (sqrtf (
                                ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                                ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                                ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                                ( input_->points[(*indices_)[i]].y - model_coefficients[1] )
                                ) - model_coefficients[2]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle2D<PointT>::selectWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold, 
    std::vector<int> &inliers)
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
    // Calculate the distance from the point to the sphere as the difference between
    // dist(point,sphere_origin) and sphere_radius
    float distance = fabsf (sqrtf (
                                  ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                                  ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                                  ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                                  ( input_->points[(*indices_)[i]].y - model_coefficients[1] )
                                  ) - model_coefficients[2]);
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
pcl::SampleConsensusModelCircle2D<PointT>::countWithinDistance (
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
    float distance = fabsf (sqrtf (
                                  ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                                  ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                                  ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                                  ( input_->points[(*indices_)[i]].y - model_coefficients[1] )
                                  ) - model_coefficients[2]);
    if (distance < threshold)
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle2D<PointT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (model_coefficients.size () != 3)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::optimizeModelCoefficients] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return;
  }

  // Need at least 3 samples
  if (inliers.size () <= 3)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::optimizeModelCoefficients] Not enough inliers found to support a model (%lu)! Returning the same coefficients.\n", inliers.size ());
    return;
  }

  tmp_inliers_ = &inliers;

  OptimizationFunctor functor (static_cast<int> (inliers.size ()), this);
  Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm (num_diff);
  int info = lm.minimize (optimized_coefficients);

  // Compute the L2 norm of the residuals
  PCL_DEBUG ("[pcl::SampleConsensusModelCircle2D::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g \nFinal solution: %g %g %g\n",
             info, lm.fvec.norm (), model_coefficients[0], model_coefficients[1], model_coefficients[2], optimized_coefficients[0], optimized_coefficients[1], optimized_coefficients[2]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle2D<PointT>::projectPoints (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
      PointCloud &projected_points, bool copy_data_fields)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 3)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::projectPoints] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return;
  }

  projected_points.header   = input_->header;
  projected_points.is_dense = input_->is_dense;

  // Copy all the data fields from the input cloud to the projected one?
  if (copy_data_fields)
  {
    // Allocate enough space and copy the basics
    projected_points.points.resize (input_->points.size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;

    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < projected_points.points.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> (input_->points[i], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      float dx = input_->points[inliers[i]].x - model_coefficients[0];
      float dy = input_->points[inliers[i]].y - model_coefficients[1];
      float a = sqrtf ( (model_coefficients[2] * model_coefficients[2]) / (dx * dx + dy * dy) );

      projected_points.points[inliers[i]].x = a * dx + model_coefficients[0];
      projected_points.points[inliers[i]].y = a * dy + model_coefficients[1];
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.points.resize (inliers.size ());
    projected_points.width    = static_cast<uint32_t> (inliers.size ());
    projected_points.height   = 1;

    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < inliers.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> (input_->points[inliers[i]], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      float dx = input_->points[inliers[i]].x - model_coefficients[0];
      float dy = input_->points[inliers[i]].y - model_coefficients[1];
      float a = sqrtf ( (model_coefficients[2] * model_coefficients[2]) / (dx * dx + dy * dy) );

      projected_points.points[i].x = a * dx + model_coefficients[0];
      projected_points.points[i].y = a * dy + model_coefficients[1];
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle2D<PointT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, const double threshold)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 3)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::doSamplesVerifyModel] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }

  for (std::set<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius
    if (fabsf (sqrtf (
                     ( input_->points[*it].x - model_coefficients[0] ) *
                     ( input_->points[*it].x - model_coefficients[0] ) +
                     ( input_->points[*it].y - model_coefficients[1] ) *
                     ( input_->points[*it].y - model_coefficients[1] )
                     ) - model_coefficients[2]) > threshold)
      return (false);

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool 
pcl::SampleConsensusModelCircle2D<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 3)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::isModelValid] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }

  if (radius_min_ != -std::numeric_limits<double>::max() && model_coefficients[2] < radius_min_)
    return (false);
  if (radius_max_ != std::numeric_limits<double>::max() && model_coefficients[2] > radius_max_)
    return (false);

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelCircle2D(T) template class PCL_EXPORTS pcl::SampleConsensusModelCircle2D<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_H_

