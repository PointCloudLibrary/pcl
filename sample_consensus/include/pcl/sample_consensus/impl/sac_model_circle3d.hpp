/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_3D_HPP_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_3D_HPP_

#include <cfloat> // for DBL_MAX

#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle3D<PointT>::isSampleGood (
    const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle3D::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }
  // Get the values at the three points
  Eigen::Vector3d p0 ((*input_)[samples[0]].x, (*input_)[samples[0]].y, (*input_)[samples[0]].z);
  Eigen::Vector3d p1 ((*input_)[samples[1]].x, (*input_)[samples[1]].y, (*input_)[samples[1]].z);
  Eigen::Vector3d p2 ((*input_)[samples[2]].x, (*input_)[samples[2]].y, (*input_)[samples[2]].z);

  // calculate vectors between points
  p1 -= p0;
  p2 -= p0;

  return (p1.dot (p2) < 0.000001);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle3D<PointT>::computeModelCoefficients (const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  // Need 3 samples
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle3D::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  model_coefficients.resize (model_size_);   //needing 7 coefficients: centerX, centerY, centerZ, radius, normalX, normalY, normalZ

  Eigen::Vector3d p0 ((*input_)[samples[0]].x, (*input_)[samples[0]].y, (*input_)[samples[0]].z);
  Eigen::Vector3d p1 ((*input_)[samples[1]].x, (*input_)[samples[1]].y, (*input_)[samples[1]].z);
  Eigen::Vector3d p2 ((*input_)[samples[2]].x, (*input_)[samples[2]].y, (*input_)[samples[2]].z);


  Eigen::Vector3d helper_vec01 = p0 - p1;
  Eigen::Vector3d helper_vec02 = p0 - p2;
  Eigen::Vector3d helper_vec10 = p1 - p0;
  Eigen::Vector3d helper_vec12 = p1 - p2;
  Eigen::Vector3d helper_vec20 = p2 - p0;
  Eigen::Vector3d helper_vec21 = p2 - p1;

  Eigen::Vector3d common_helper_vec = helper_vec01.cross (helper_vec12);

  double commonDividend = 2.0 * common_helper_vec.squaredNorm ();

  double alpha = (helper_vec12.squaredNorm () * helper_vec01.dot (helper_vec02)) / commonDividend;
  double beta =  (helper_vec02.squaredNorm () * helper_vec10.dot (helper_vec12)) / commonDividend;
  double gamma = (helper_vec01.squaredNorm () * helper_vec20.dot (helper_vec21)) / commonDividend;

  Eigen::Vector3d circle_center = alpha * p0 + beta * p1 + gamma * p2;

  Eigen::Vector3d circle_radiusVector = circle_center - p0;
  double circle_radius = circle_radiusVector.norm ();
  Eigen::Vector3d circle_normal = common_helper_vec.normalized (); 

  model_coefficients[0] = static_cast<float> (circle_center[0]);
  model_coefficients[1] = static_cast<float> (circle_center[1]);
  model_coefficients[2] = static_cast<float> (circle_center[2]);
  model_coefficients[3] = static_cast<float> (circle_radius);
  model_coefficients[4] = static_cast<float> (circle_normal[0]);
  model_coefficients[5] = static_cast<float> (circle_normal[1]);
  model_coefficients[6] = static_cast<float> (circle_normal[2]);

  PCL_DEBUG ("[pcl::SampleConsensusModelCircle3D::computeModelCoefficients] Model is (%g,%g,%g,%g,%g,%g,%g).\n",
             model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3],
             model_coefficients[4], model_coefficients[5], model_coefficients[6]);
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle3D<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
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
  // Calculate the distance from the point to the circle:
  // 1.   calculate intersection point of the plane in which the circle lies and the
  //      line from the sample point with the direction of the plane normal (projected point)
  // 2.   calculate the intersection point of the line from the circle center to the projected point
  //      with the circle
  // 3.   calculate distance from corresponding point on the circle to the sample point
  {
    // what i have:
    // P : Sample Point
    Eigen::Vector3d P ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z);
    // C : Circle Center
    Eigen::Vector3d C (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    // N : Circle (Plane) Normal
    Eigen::Vector3d N (model_coefficients[4], model_coefficients[5], model_coefficients[6]);
    // r : Radius
    double r = model_coefficients[3];

    Eigen::Vector3d helper_vectorPC = P - C;
    // 1.1. get line parameter
    double lambda = (helper_vectorPC.dot (N)) / N.squaredNorm ();

    // Projected Point on plane
    Eigen::Vector3d P_proj = P + lambda * N;
    Eigen::Vector3d helper_vectorP_projC = P_proj - C;

    // K : Point on Circle
    Eigen::Vector3d K = C + r * helper_vectorP_projC.normalized ();
    Eigen::Vector3d distanceVector =  P - K;

    distances[i] = distanceVector.norm ();
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle3D<PointT>::selectWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold,
    Indices &inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }
  inliers.clear ();
  inliers.reserve (indices_->size ());

  const auto squared_threshold = threshold * threshold;
  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // what i have:
    // P : Sample Point
    Eigen::Vector3d P ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z);
    // C : Circle Center
    Eigen::Vector3d C (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    // N : Circle (Plane) Normal
    Eigen::Vector3d N (model_coefficients[4], model_coefficients[5], model_coefficients[6]);
    // r : Radius
    double r = model_coefficients[3];

    Eigen::Vector3d helper_vectorPC = P - C;
    // 1.1. get line parameter
    double lambda = (-(helper_vectorPC.dot (N))) / N.dot (N);
    // Projected Point on plane
    Eigen::Vector3d P_proj = P + lambda * N;
    Eigen::Vector3d helper_vectorP_projC = P_proj - C;

    // K : Point on Circle
    Eigen::Vector3d K = C + r * helper_vectorP_projC.normalized ();
    Eigen::Vector3d distanceVector =  P - K;

    if (distanceVector.squaredNorm () < squared_threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers.push_back ((*indices_)[i]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelCircle3D<PointT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);
  std::size_t nr_p = 0;

  const auto squared_threshold = threshold * threshold;
  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // what i have:
    // P : Sample Point
    Eigen::Vector3d P ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z);
    // C : Circle Center
    Eigen::Vector3d C (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    // N : Circle (Plane) Normal
    Eigen::Vector3d N (model_coefficients[4], model_coefficients[5], model_coefficients[6]);
    // r : Radius
    double r = model_coefficients[3];

    Eigen::Vector3d helper_vectorPC = P - C;
    // 1.1. get line parameter
    double lambda = (-(helper_vectorPC.dot (N))) / N.dot (N);

    // Projected Point on plane
    Eigen::Vector3d P_proj = P + lambda * N;
    Eigen::Vector3d helper_vectorP_projC = P_proj - C;

    // K : Point on Circle
    Eigen::Vector3d K = C + r * helper_vectorP_projC.normalized ();
    Eigen::Vector3d distanceVector =  P - K;

    if (distanceVector.squaredNorm () < squared_threshold)
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle3D<PointT>::optimizeModelCoefficients (
      const Indices &inliers,
      const Eigen::VectorXf &model_coefficients,
      Eigen::VectorXf &optimized_coefficients) const
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle3D::optimizeModelCoefficients] Given model is invalid!\n");
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size () <= sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle3D::optimizeModelCoefficients] Not enough inliers to refine/optimize the model's coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
    return;
  }

  OptimizationFunctor functor (this, inliers);
  Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm (num_diff);
  Eigen::VectorXd coeff;
  int info = lm.minimize (coeff);
  for (Eigen::Index i = 0; i < coeff.size (); ++i)
    optimized_coefficients[i] = static_cast<float> (coeff[i]);

  // Compute the L2 norm of the residuals
  PCL_DEBUG ("[pcl::SampleConsensusModelCircle3D::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g\n",
             info, lm.fvec.norm (), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3], model_coefficients[4], model_coefficients[5], model_coefficients[6], optimized_coefficients[0], optimized_coefficients[1], optimized_coefficients[2], optimized_coefficients[3], optimized_coefficients[4], optimized_coefficients[5], optimized_coefficients[6]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle3D<PointT>::projectPoints (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients,
      PointCloud &projected_points, bool copy_data_fields) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle3D::projectPoints] Given model is invalid!\n");
    return;
  }

  projected_points.header   = input_->header;
  projected_points.is_dense = input_->is_dense;

  // Copy all the data fields from the input cloud to the projected one?
  if (copy_data_fields)
  {
    // Allocate enough space and copy the basics
    projected_points.resize (input_->size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < projected_points.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[i], projected_points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      // what i have:
      // P : Sample Point
      Eigen::Vector3d P ((*input_)[inliers[i]].x, (*input_)[inliers[i]].y, (*input_)[inliers[i]].z);
      // C : Circle Center
      Eigen::Vector3d C (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
      // N : Circle (Plane) Normal
      Eigen::Vector3d N (model_coefficients[4], model_coefficients[5], model_coefficients[6]);
      // r : Radius
      double r = model_coefficients[3];

      Eigen::Vector3d helper_vectorPC = P - C;
      // 1.1. get line parameter
      //float lambda = (helper_vectorPC.dot(N)) / N.squaredNorm() ;
      double lambda = (-(helper_vectorPC.dot (N))) / N.dot (N);
      // Projected Point on plane
      Eigen::Vector3d P_proj = P + lambda * N;
      Eigen::Vector3d helper_vectorP_projC = P_proj - C;

      // K : Point on Circle
      Eigen::Vector3d K = C + r * helper_vectorP_projC.normalized ();

      projected_points[i].x = static_cast<float> (K[0]);
      projected_points[i].y = static_cast<float> (K[1]);
      projected_points[i].z = static_cast<float> (K[2]);
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.resize (inliers.size ());
    projected_points.width    = inliers.size ();
    projected_points.height   = 1;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < inliers.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[inliers[i]], projected_points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      // what i have:
      // P : Sample Point
      Eigen::Vector3d P ((*input_)[inliers[i]].x, (*input_)[inliers[i]].y, (*input_)[inliers[i]].z);
      // C : Circle Center
      Eigen::Vector3d C (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
      // N : Circle (Plane) Normal
      Eigen::Vector3d N (model_coefficients[4], model_coefficients[5], model_coefficients[6]);
      // r : Radius
      double r = model_coefficients[3];

      Eigen::Vector3d helper_vectorPC = P - C;
      // 1.1. get line parameter
      double lambda = (-(helper_vectorPC.dot (N))) / N.dot (N);
      // Projected Point on plane
      Eigen::Vector3d P_proj = P + lambda * N;
      Eigen::Vector3d helper_vectorP_projC = P_proj - C;

      // K : Point on Circle
      Eigen::Vector3d K = C + r * helper_vectorP_projC.normalized ();

      projected_points[i].x = static_cast<float> (K[0]);
      projected_points[i].y = static_cast<float> (K[1]);
      projected_points[i].z = static_cast<float> (K[2]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle3D<PointT>::doSamplesVerifyModel (
      const std::set<index_t> &indices,
      const Eigen::VectorXf &model_coefficients,
      const double threshold) const
{
  // Needs a valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle3D::doSamplesVerifyModel] Given model is invalid!\n");
    return (false);
  }

  const auto squared_threshold = threshold * threshold;
  for (const auto &index : indices)
  {
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius

    // what i have:
    // P : Sample Point
    Eigen::Vector3d P ((*input_)[index].x, (*input_)[index].y, (*input_)[index].z);
    // C : Circle Center
    Eigen::Vector3d C (model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    // N : Circle (Plane) Normal
    Eigen::Vector3d N (model_coefficients[4], model_coefficients[5], model_coefficients[6]);
    // r : Radius
    double r = model_coefficients[3];
    Eigen::Vector3d helper_vectorPC = P - C;
    // 1.1. get line parameter
    double lambda = (-(helper_vectorPC.dot (N))) / N.dot (N);
    // Projected Point on plane
    Eigen::Vector3d P_proj = P + lambda * N;
    Eigen::Vector3d helper_vectorP_projC = P_proj - C;

    // K : Point on Circle
    Eigen::Vector3d K = C + r * helper_vectorP_projC.normalized ();
    Eigen::Vector3d distanceVector =  P - K;

    if (distanceVector.squaredNorm () > squared_threshold)
      return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle3D<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
  if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
    return (false);

  if (radius_min_ != -DBL_MAX && model_coefficients[3] < radius_min_)
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelCircle3D::isModelValid] Radius of circle is too small: should be larger than %g, but is %g.\n",
               radius_min_, model_coefficients[3]);
    return (false);
  }
  if (radius_max_ != DBL_MAX && model_coefficients[3] > radius_max_)
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelCircle3D::isModelValid] Radius of circle is too big: should be smaller than %g, but is %g.\n",
               radius_max_, model_coefficients[3]);
    return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelCircle3D(T) template class PCL_EXPORTS pcl::SampleConsensusModelCircle3D<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE3D_HPP_

