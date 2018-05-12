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
 * $Id: sac_model_line.hpp 2328 2011-08-31 08:11:00Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_STICK_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_STICK_H_

#include <pcl/sample_consensus/sac_model_stick.h>
#include <pcl/common/centroid.h>
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelStick<PointT>::isSampleGood (const std::vector<int> &samples) const
{
  if (
      (input_->points[samples[0]].x != input_->points[samples[1]].x)
    &&
      (input_->points[samples[0]].y != input_->points[samples[1]].y)
    &&
      (input_->points[samples[0]].z != input_->points[samples[1]].z))
    return (true);

  return (false);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelStick<PointT>::computeModelCoefficients (
      const std::vector<int> &samples, Eigen::VectorXf &model_coefficients) const
{
  // Need 2 samples
  if (samples.size () != 2)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelStick::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  model_coefficients.resize (7);
  model_coefficients[0] = input_->points[samples[0]].x;
  model_coefficients[1] = input_->points[samples[0]].y;
  model_coefficients[2] = input_->points[samples[0]].z;

  model_coefficients[3] = input_->points[samples[1]].x;
  model_coefficients[4] = input_->points[samples[1]].y;
  model_coefficients[5] = input_->points[samples[1]].z;

//  model_coefficients[3] = input_->points[samples[1]].x - model_coefficients[0];
//  model_coefficients[4] = input_->points[samples[1]].y - model_coefficients[1];
//  model_coefficients[5] = input_->points[samples[1]].z - model_coefficients[2];

//  model_coefficients.template segment<3> (3).normalize ();
  // We don't care about model_coefficients[6] which is the width (radius) of the stick

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelStick<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
    return;

  float sqr_threshold = static_cast<float> (radius_max_ * radius_max_);
  distances.resize (indices_->size ());

  // Obtain the line point and direction
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  line_dir.normalize ();

  // Iterate through the 3d points and calculate the distances from them to the line
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
    float sqr_distance = (line_pt - input_->points[(*indices_)[i]].getVector4fMap ()).cross3 (line_dir).squaredNorm ();

    if (sqr_distance < sqr_threshold)
      // Need to estimate sqrt here to keep MSAC and friends general
      distances[i] = sqrt (sqr_distance);
    else
      // Penalize outliers by doubling the distance
      distances[i] = 2 * sqrt (sqr_distance);
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelStick<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
    return;

  float sqr_threshold = static_cast<float> (threshold * threshold);

  int nr_p = 0;
  inliers.resize (indices_->size ());
  error_sqr_dists_.resize (indices_->size ());

  // Obtain the line point and direction
  Eigen::Vector4f line_pt1 (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_pt2 (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  Eigen::Vector4f line_dir = line_pt2 - line_pt1;
  //Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  //Eigen::Vector4f line_dir (model_coefficients[3] - model_coefficients[0], model_coefficients[4] - model_coefficients[1], model_coefficients[5] - model_coefficients[2], 0);
  line_dir.normalize ();
  //float norm = line_dir.squaredNorm ();

  // Iterate through the 3d points and calculate the distances from them to the line
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
    Eigen::Vector4f dir = input_->points[(*indices_)[i]].getVector4fMap () - line_pt1;
    //float u = dir.dot (line_dir);

    // If the point falls outside of the segment, ignore it
    //if (u < 0.0f || u > 1.0f)
    //  continue;

    float sqr_distance = dir.cross3 (line_dir).squaredNorm ();
    if (sqr_distance < sqr_threshold)
    {
      // Returns the indices of the points whose squared distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      error_sqr_dists_[nr_p] = static_cast<double> (sqr_distance);
      ++nr_p;
    }
  }
  inliers.resize (nr_p);
  error_sqr_dists_.resize (nr_p);
}

///////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::SampleConsensusModelStick<PointT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
    return (0);

  float sqr_threshold = static_cast<float> (threshold * threshold);

  int nr_i = 0, nr_o = 0;

  // Obtain the line point and direction
  Eigen::Vector4f line_pt1 (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_pt2 (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  Eigen::Vector4f line_dir = line_pt2 - line_pt1;
  line_dir.normalize ();

  //Eigen::Vector4f line_dir (model_coefficients[3] - model_coefficients[0], model_coefficients[4] - model_coefficients[1], model_coefficients[5] - model_coefficients[2], 0);
  //Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

  // Iterate through the 3d points and calculate the distances from them to the line
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
    Eigen::Vector4f dir = input_->points[(*indices_)[i]].getVector4fMap () - line_pt1;
    //float u = dir.dot (line_dir);

    // If the point falls outside of the segment, ignore it
    //if (u < 0.0f || u > 1.0f)
    //  continue;

    float sqr_distance = dir.cross3 (line_dir).squaredNorm ();
    // Use a larger threshold (4 times the radius) to get more points in
    if (sqr_distance < sqr_threshold)
      nr_i++;
    else if (sqr_distance < 4 * sqr_threshold)
      nr_o++;
  }

  return (nr_i - nr_o < 0 ? 0 : nr_i - nr_o);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelStick<PointT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    optimized_coefficients = model_coefficients;
    return;
  }

  // Need at least 2 points to estimate a line
  if (inliers.size () <= 2)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelStick::optimizeModelCoefficients] Not enough inliers found to support a model (%lu)! Returning the same coefficients.\n", inliers.size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  optimized_coefficients.resize (7);

  // Compute the 3x3 covariance matrix
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;

  computeMeanAndCovarianceMatrix (*input_, inliers, covariance_matrix, centroid);

  optimized_coefficients[0] = centroid[0];
  optimized_coefficients[1] = centroid[1];
  optimized_coefficients[2] = centroid[2];

  // Extract the eigenvalues and eigenvectors
  Eigen::Vector3f eigen_values;
  Eigen::Vector3f eigen_vector;
  pcl::eigen33 (covariance_matrix, eigen_values);
  pcl::computeCorrespondingEigenVector (covariance_matrix, eigen_values [2], eigen_vector);

  optimized_coefficients.template segment<3> (3).matrix () = eigen_vector;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelStick<PointT>::projectPoints (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields) const
{
  // Needs a valid model coefficients
  if (!isModelValid (model_coefficients))
    return;

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
      float k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);

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
    projected_points.width    = static_cast<uint32_t> (inliers.size ());
    projected_points.height   = 1;

    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < inliers.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> (input_->points[inliers[i]], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the line
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      Eigen::Vector4f pt (input_->points[inliers[i]].x, input_->points[inliers[i]].y, input_->points[inliers[i]].z, 0);
      // double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
      float k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);

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
pcl::SampleConsensusModelStick<PointT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
    return (false);

  // Obtain the line point and direction
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3] - model_coefficients[0], model_coefficients[4] - model_coefficients[1], model_coefficients[5] - model_coefficients[2], 0);
  //Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  line_dir.normalize ();

  float sqr_threshold = static_cast<float> (threshold * threshold);
  // Iterate through the 3d points and calculate the distances from them to the line
  for (std::set<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
  {
    // Calculate the distance from the point to the line
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
    if ((line_pt - input_->points[*it].getVector4fMap ()).cross3 (line_dir).squaredNorm () > sqr_threshold)
      return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelStick(T) template class PCL_EXPORTS pcl::SampleConsensusModelStick<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_STICK_H_

