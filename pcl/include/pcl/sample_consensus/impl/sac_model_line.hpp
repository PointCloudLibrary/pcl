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
 * $Id: sac_model_line.hpp 34403 2010-12-01 01:48:52Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_LINE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_LINE_H_

#include "pcl/sample_consensus/sac_model_line.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelLine<PointT>::getSamples (int &iterations, std::vector<int> &samples)
{
  // We're assuming that indices_ have already been set in the constructor
  if (indices_->empty ())
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::getSamples] Empty set of indices given!");
    return;
  }

  samples.resize (2);
  double trand = indices_->size () / (RAND_MAX + 1.0);

  // Check if we have enough points
  if (samples.size () > indices_->size ())
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::getSamples] Can not select %zu unique points out of %zu!", samples.size (), indices_->size ());
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
  int iter = 0;
  do
  {
    idx = (int)(rand () * trand);
    samples[1] = (*indices_)[idx];
    ++iter;

    if (iter > MAX_ITERATIONS_UNIQUE )
    {
      ROS_DEBUG ("[pcl::SampleConsensusModelLine::getSamples] WARNING: Could not select 2 unique points in %d iterations (%zu indices)!", MAX_ITERATIONS_UNIQUE, indices_->size ());
      break;
    }
    //iterations++;
  }
  while (samples[1] == samples[0]);
  //iterations--;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelLine<PointT>::computeModelCoefficients (
      const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 2 samples
  if (samples.size () != 2)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::computeModelCoefficients] Invalid set of samples given (%zu)!", samples.size ());
    return (false);
  }

  model_coefficients.resize (6);
  model_coefficients[0] = input_->points[samples[0]].x;
  model_coefficients[1] = input_->points[samples[0]].y;
  model_coefficients[2] = input_->points[samples[0]].z;

  model_coefficients[3] = input_->points[samples[1]].x - model_coefficients[0];
  model_coefficients[4] = input_->points[samples[1]].y - model_coefficients[1];
  model_coefficients[5] = input_->points[samples[1]].z - model_coefficients[2];

  model_coefficients.template tail<3> ().normalize ();
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelLine<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 6)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::getDistancesToModel] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  distances.resize (indices_->size ());

  // Obtain the line point and direction
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  double line_dir_squaredNorm = line_dir.squaredNorm ();

  // Iterate through the 3d points and calculate the distances from them to the line
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    // Need to estimate sqrt here to keep MSAC and friends general
    distances[i] = sqrt ((line_dir.cross3 (line_pt - pt)).squaredNorm () / line_dir_squaredNorm);
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelLine<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 6)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::selectWithinDistance] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  double sqr_threshold = threshold * threshold;

  int nr_p = 0;
  inliers.resize (indices_->size ());

  // Obtain the line point and direction
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  Eigen::Vector4f line_p2 = line_pt + line_dir;

  // Iterate through the 3d points and calculate the distances from them to the line
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f pp = line_p2 - pt;

    Eigen::Vector4f c = pp.cross3 (line_dir);
    c[3] = 0;
    //distances[i] = sqrt (c.dot (c)) / line_dir.dot (line_dir);
    double sqr_distance = c.dot (c) / line_dir.dot (line_dir);

    if (sqr_distance < sqr_threshold)
    {
      // Returns the indices of the points whose squared distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      nr_p++;
    }
  }
  inliers.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelLine<PointT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 6)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::optimizeModelCoefficients] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  // Need at least 2 points to estimate a line
  if (inliers.size () <= 2)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::optimizeModelCoefficients] Not enough inliers found to support a model (%zu)! Returning the same coefficients.", inliers.size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  optimized_coefficients.resize (6);

  // Compute the 3x3 covariance matrix
  Eigen::Vector4f centroid;
  compute3DCentroid (*input_, inliers, centroid);
  Eigen::Matrix3f covariance_matrix;
  computeCovarianceMatrix (*input_, inliers, centroid, covariance_matrix);
  optimized_coefficients[0] = centroid[0];
  optimized_coefficients[1] = centroid[1];
  optimized_coefficients[2] = centroid[2];

  // Extract the eigenvalues and eigenvectors
  //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
  //EIGEN_ALIGN16 Eigen::Vector3f eigen_values  = ei_symm.eigenvalues ();
  //EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors = ei_symm.eigenvectors ();
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  optimized_coefficients.template tail<3> () = eigen_vectors.col (2).normalized ();
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelLine<PointT>::projectPoints (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 6)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::projectPoints] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  // Obtain the line point and direction
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

  projected_points.header = input_->header;
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

    // Iterate through the 3d points and calculate the distances from them to the line
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      Eigen::Vector4f pt (input_->points[inliers[i]].x, input_->points[inliers[i]].y, input_->points[inliers[i]].z, 0);
      // double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
      double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);

      Eigen::Vector4f pp = line_pt + k * line_dir;
      // Calculate the projection of the point on the line (pointProj = A + k * B)
      projected_points.points[inliers[i]].x = pp[0];
      projected_points.points[inliers[i]].y = pp[1];
      projected_points.points[inliers[i]].z = pp[2];
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.points.resize (inliers.size ());
    projected_points.width    = inliers.size ();
    projected_points.height   = 1;

    // Iterate through the 3d points and calculate the distances from them to the line
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      Eigen::Vector4f pt (input_->points[inliers[i]].x, input_->points[inliers[i]].y, input_->points[inliers[i]].z, 0);
      // double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
      double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);

      Eigen::Vector4f pp = line_pt + k * line_dir;
      // Calculate the projection of the point on the line (pointProj = A + k * B)
      projected_points.points[i].x = pp[0];
      projected_points.points[i].y = pp[1];
      projected_points.points[i].z = pp[2];
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelLine<PointT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 6)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelLine::doSamplesVerifyModel] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return (false);
  }

  // Obtain the line point and direction
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  Eigen::Vector4f line_p2 = line_pt + line_dir;

  double sqr_threshold = threshold * threshold;
  // Iterate through the 3d points and calculate the distances from them to the line
  for (std::set<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
    Eigen::Vector4f pt (input_->points[*it].x, input_->points[*it].y, input_->points[*it].z, 0);
    Eigen::Vector4f pp = line_p2 - pt;

    Eigen::Vector4f c = pp.cross3 (line_dir);
    c[3] = 0;
    double sqr_distance = c.dot (c) / line_dir.dot (line_dir);

    if (sqr_distance > sqr_threshold)
      return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelLine(T) template class pcl::SampleConsensusModelLine<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_LINE_H_

