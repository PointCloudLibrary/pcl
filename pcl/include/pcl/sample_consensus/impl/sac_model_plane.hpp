/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: sac_model_plane.hpp 34403 2010-12-01 01:48:52Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PLANE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PLANE_H_

#include "pcl/sample_consensus/sac_model_plane.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::getSamples (int &iterations, std::vector<int> &samples)
{
  // We're assuming that indices_ have already been set in the constructor
  if (indices_->empty ())
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::getSamples] Empty set of indices given!");
    return;
  }

  samples.resize (3);
  double trand = indices_->size () / (RAND_MAX + 1.0);

  // Check if we have enough points
  if (samples.size () > indices_->size ())
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::getSamples] Can not select %zu unique points out of %zu!", samples.size (), indices_->size ());
    // one of these will make it stop :) TODO static constant for each model that the method has to check
    samples.clear ();
    iterations = INT_MAX - 1;
    return;
  }

  // Get a random number between 1 and max_indices
  int idx = (int)(rand () * trand);
  // Get the index
  samples[0] = (*indices_)[idx];

  // Get a second point which is different than the first
  do
  {
    idx = (int)(rand () * trand);
    samples[1] = (*indices_)[idx];
    //iterations++;
  } while (samples[1] == samples[0]);
  //iterations--;

  // Get the values at the two points
  pcl::Array4fMapConst p0 = input_->points[samples[0]].getArray4fMap ();
  pcl::Array4fMapConst p1 = input_->points[samples[1]].getArray4fMap ();

  // Compute the segment values (in 3d) between p1 and p0
  Eigen::Array4f p1p0 = p1 - p0;

  Eigen::Array4f dy1dy2;
  int iter = 0;
  do
  {
    // Get the third point, different from the first two
    do
    {
      idx = (int)(rand () * trand);
      samples[2] = (*indices_)[idx];
      //iterations++;
    } while ( (samples[2] == samples[1]) || (samples[2] == samples[0]) );
    //iterations--;

    pcl::Array4fMapConst p2 = input_->points[samples[2]].getArray4fMap ();

    // Compute the segment values (in 3d) between p2 and p0
    Eigen::Array4f p2p0 = p2 - p0;

    dy1dy2 = p1p0 / p2p0;
    ++iter;
    if (iter > MAX_ITERATIONS_COLLINEAR )
    {
      ROS_DEBUG ("[pcl::SampleConsensusModelPlane::getSamples] WARNING: Could not select 3 non collinear points in %d iterations!", MAX_ITERATIONS_COLLINEAR);
      break;
    }
    //iterations++;
  }
  while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
  //iterations--;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelPlane<PointT>::computeModelCoefficients (
      const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 3 samples
  if (samples.size () != 3)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::computeModelCoefficients] Invalid set of samples given (%zu)!", samples.size ());
    return (false);
  }

  pcl::Array4fMapConst p0 = input_->points[samples[0]].getArray4fMap ();
  pcl::Array4fMapConst p1 = input_->points[samples[1]].getArray4fMap ();
  pcl::Array4fMapConst p2 = input_->points[samples[2]].getArray4fMap ();

  // Compute the segment values (in 3d) between p1 and p0
  Eigen::Array4f p1p0 = p1 - p0;
  // Compute the segment values (in 3d) between p2 and p0
  Eigen::Array4f p2p0 = p2 - p0;

  // Avoid some crashes by checking for collinearity here
  Eigen::Array4f dy1dy2 = p1p0 / p2p0;
  if ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) )          // Check for collinearity
    return (false);

  // Compute the plane coefficients from the 3 given points in a straightforward manner
  // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
  model_coefficients.resize (4);
  model_coefficients[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
  model_coefficients[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
  model_coefficients[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];
  model_coefficients[3] = 0;

  // Normalize
  model_coefficients.normalize ();

  // ... + d = 0
  model_coefficients[3] = -1 * (model_coefficients.template head<4>().dot (p0.matrix ()));

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::getDistancesToModel] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    /*distances[i] = fabs (model_coefficients[0] * input_->points[(*indices_)[i]].x +
                         model_coefficients[1] * input_->points[(*indices_)[i]].y +
                         model_coefficients[2] * input_->points[(*indices_)[i]].z +
                         model_coefficients[3]);*/
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x,
                        input_->points[(*indices_)[i]].y,
                        input_->points[(*indices_)[i]].z,
                        1);
    distances[i] = fabs (model_coefficients.dot (pt));
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::selectWithinDistance] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  int nr_p = 0;
  inliers.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x,
                        input_->points[(*indices_)[i]].y,
                        input_->points[(*indices_)[i]].z,
                        1);
    if (fabs (model_coefficients.dot (pt)) < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      nr_p++;
    }
  }
  inliers.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::optimizeModelCoefficients] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  // Need at least 3 points to estimate a plane
  if (inliers.size () < 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::optimizeModelCoefficients] Not enough inliers found to support a model (%zu)! Returning the same coefficients.", inliers.size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  Eigen::Vector4f plane_parameters;
  float curvature;

  // Use Least-Squares to fit the plane through all the given sample points and find out its coefficients
  computePointNormal (*input_, inliers, plane_parameters, curvature);
  optimized_coefficients = plane_parameters;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::projectPoints (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::projectPoints] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  projected_points.header = input_->header;
  projected_points.is_dense = input_->is_dense;

  Eigen::Vector4f mc (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
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
      // Calculate the distance from the point to the plane
      Eigen::Vector4f p (input_->points[inliers[i]].x,
                         input_->points[inliers[i]].y,
                         input_->points[inliers[i]].z,
                         1);
      float distance_to_plane = model_coefficients.dot (p);

      pcl::Vector4fMap pp = projected_points.points[inliers[i]].getVector4fMap ();
      pp = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.points.resize (inliers.size ());
    projected_points.width    = inliers.size ();
    projected_points.height   = 1;

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      // Calculate the distance from the point to the plane
      Eigen::Vector4f p (input_->points[inliers[i]].x,
                         input_->points[inliers[i]].y,
                         input_->points[inliers[i]].z,
                         1);
      float distance_to_plane = model_coefficients.dot (p);

      pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
      pp = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelPlane<PointT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelPlane::doSamplesVerifyModel] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return (false);
  }

  for (std::set<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
  {
    Eigen::Vector4f pt (input_->points[*it].x,
                        input_->points[*it].y,
                        input_->points[*it].z,
                        1);
    if (fabs (model_coefficients.dot (pt)) > threshold)
      return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelPlane(T) template class pcl::SampleConsensusModelPlane<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PLANE_H_

