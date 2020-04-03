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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PLANE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PLANE_H_

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/concatenate.h>
#include <pcl/point_types.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelPlane<PointT>::isSampleGood (const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }
  // Get the values at the two points
  pcl::Array4fMapConst p0 = input_->points[samples[0]].getArray4fMap ();
  pcl::Array4fMapConst p1 = input_->points[samples[1]].getArray4fMap ();
  pcl::Array4fMapConst p2 = input_->points[samples[2]].getArray4fMap ();

  Eigen::Array4f dy1dy2 = (p1-p0) / (p2-p0);

  return ( (dy1dy2[0] != dy1dy2[1]) || (dy1dy2[2] != dy1dy2[1]) );
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelPlane<PointT>::computeModelCoefficients (
      const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  // Need 3 samples
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
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
  {
    return (false);
  }

  // Compute the plane coefficients from the 3 given points in a straightforward manner
  // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
  model_coefficients.resize (model_size_);
  model_coefficients[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
  model_coefficients[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
  model_coefficients[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];
  model_coefficients[3] = 0.0f;

  // Normalize
  model_coefficients.normalize ();

  // ... + d = 0
  model_coefficients[3] = -1.0f * (model_coefficients.template head<4>().dot (p0.matrix ()));

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::getDistancesToModel] Given model is invalid!\n");
    return;
  }

  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    /*distances[i] = std::abs (model_coefficients[0] * input_->points[(*indices_)[i]].x +
                         model_coefficients[1] * input_->points[(*indices_)[i]].y +
                         model_coefficients[2] * input_->points[(*indices_)[i]].z +
                         model_coefficients[3]);*/
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x,
                        input_->points[(*indices_)[i]].y,
                        input_->points[(*indices_)[i]].z,
                        1.0f);
    distances[i] = std::abs (model_coefficients.dot (pt));
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::selectWithinDistance] Given model is invalid!\n");
    return;
  }

  inliers.clear ();
  error_sqr_dists_.clear ();
  inliers.reserve (indices_->size ());
  error_sqr_dists_.reserve (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x,
                        input_->points[(*indices_)[i]].y,
                        input_->points[(*indices_)[i]].z,
                        1.0f);
    
    float distance = std::abs (model_coefficients.dot (pt));
    
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
pcl::SampleConsensusModelPlane<PointT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::countWithinDistance] Given model is invalid!\n");
    return (0);
  }

  std::size_t nr_p = 0;

  // Iterate through the 3d points and calculate the distances from them to the plane
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x,
                        input_->points[(*indices_)[i]].y,
                        input_->points[(*indices_)[i]].z,
                        1.0f);
    if (std::abs (model_coefficients.dot (pt)) < threshold)
    {
      nr_p++;
    }
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::optimizeModelCoefficients (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::optimizeModelCoefficients] Given model is invalid!\n");
    optimized_coefficients = model_coefficients;
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size () <= sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::optimizeModelCoefficients] Not enough inliers found to optimize model coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  Eigen::Vector4f plane_parameters;

  // Use Least-Squares to fit the plane through all the given sample points and find out its coefficients
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;

  computeMeanAndCovarianceMatrix (*input_, inliers, covariance_matrix, xyz_centroid);

  // Compute the model coefficients
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);

  // Hessian form (D = nc . p_plane (centroid here) + p)
  optimized_coefficients.resize (model_size_);
  optimized_coefficients[0] = eigen_vector [0];
  optimized_coefficients[1] = eigen_vector [1];
  optimized_coefficients[2] = eigen_vector [2];
  optimized_coefficients[3] = 0.0f;
  optimized_coefficients[3] = -1.0f * optimized_coefficients.dot (xyz_centroid);

  // Make sure it results in a valid model
  if (!isModelValid (optimized_coefficients))
  {
    optimized_coefficients = model_coefficients;
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPlane<PointT>::projectPoints (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::projectPoints] Given model is invalid!\n");
    return;
  }

  projected_points.header = input_->header;
  projected_points.is_dense = input_->is_dense;

  Eigen::Vector4f mc (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);

  // normalize the vector perpendicular to the plane...
  mc.normalize ();
  // ... and store the resulting normal as a local copy of the model coefficients
  Eigen::Vector4f tmp_mc = model_coefficients;
  tmp_mc[0] = mc[0];
  tmp_mc[1] = mc[1];
  tmp_mc[2] = mc[2];

  // Copy all the data fields from the input cloud to the projected one?
  if (copy_data_fields)
  {
    // Allocate enough space and copy the basics
    projected_points.points.resize (input_->points.size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < input_->points.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> (input_->points[i], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (const auto &inlier : inliers)
    {
      // Calculate the distance from the point to the plane
      Eigen::Vector4f p (input_->points[inlier].x,
                         input_->points[inlier].y,
                         input_->points[inlier].z,
                         1);
      // use normalized coefficients to calculate the scalar projection
      float distance_to_plane = tmp_mc.dot (p);

      pcl::Vector4fMap pp = projected_points.points[inlier].getVector4fMap ();
      pp.matrix () = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.points.resize (inliers.size ());
    projected_points.width    = static_cast<std::uint32_t> (inliers.size ());
    projected_points.height   = 1;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> (input_->points[inliers[i]], projected_points.points[i]));
    }

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      // Calculate the distance from the point to the plane
      Eigen::Vector4f p (input_->points[inliers[i]].x,
                         input_->points[inliers[i]].y,
                         input_->points[inliers[i]].z,
                         1.0f);
      // use normalized coefficients to calculate the scalar projection
      float distance_to_plane = tmp_mc.dot (p);

      pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
      pp.matrix () = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelPlane<PointT>::doSamplesVerifyModel (
      const std::set<index_t> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::doSamplesVerifyModel] Given model is invalid!\n");
    return (false);
  }

  for (const auto &index : indices)
  {
    Eigen::Vector4f pt (input_->points[index].x,
                        input_->points[index].y,
                        input_->points[index].z,
                        1.0f);
    if (std::abs (model_coefficients.dot (pt)) > threshold)
    {
      return (false);
    }
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelPlane(T) template class PCL_EXPORTS pcl::SampleConsensusModelPlane<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PLANE_H_

