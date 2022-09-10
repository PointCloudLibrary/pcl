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

#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle2D<PointT>::isSampleGood(const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }

  // Double precision here follows computeModelCoefficients, which means we
  // can't use getVector3fMap-accessor to make our lives easier.
  Eigen::Array2d p0 ((*input_)[samples[0]].x, (*input_)[samples[0]].y);
  Eigen::Array2d p1 ((*input_)[samples[1]].x, (*input_)[samples[1]].y);
  Eigen::Array2d p2 ((*input_)[samples[2]].x, (*input_)[samples[2]].y);

  // Compute the segment values (in 2d) between p1 and p0
  p1 -= p0;
  // Compute the segment values (in 2d) between p2 and p0
  p2 -= p0;

  // Check if the triangle area spanned by the three sample points is greater than 0
  // https://en.wikipedia.org/wiki/Triangle#Using_vectors
  if (std::abs (p1.x()*p2.y() - p2.x()*p1.y()) < Eigen::NumTraits<double>::dummy_precision ()) {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::isSampleGood] Sample points too similar or collinear!\n");
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle2D<PointT>::computeModelCoefficients (const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  if (!isSampleGood (samples))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::computeModelCoefficients] Invalid set of samples given!\n");
    return (false);
  }

  model_coefficients.resize (model_size_);

  Eigen::Vector2d p0 ((*input_)[samples[0]].x, (*input_)[samples[0]].y);
  Eigen::Vector2d p1 ((*input_)[samples[1]].x, (*input_)[samples[1]].y);
  Eigen::Vector2d p2 ((*input_)[samples[2]].x, (*input_)[samples[2]].y);

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
  PCL_DEBUG ("[pcl::SampleConsensusModelCircle2D::computeModelCoefficients] Model is (%g,%g,%g).\n",
             model_coefficients[0], model_coefficients[1], model_coefficients[2]);
  return (true);
}

#define AT(POS) ((*input_)[(*indices_)[(POS)]])

#ifdef __AVX__
// This function computes the squared distances (i.e. the distances without the square root) of 8 points to the center of the circle
template <typename PointT> inline __m256 pcl::SampleConsensusModelCircle2D<PointT>::sqr_dist8 (const std::size_t i, const __m256 a_vec, const __m256 b_vec) const
{
  const __m256 tmp1 = _mm256_sub_ps (_mm256_set_ps (AT(i  ).x, AT(i+1).x, AT(i+2).x, AT(i+3).x, AT(i+4).x, AT(i+5).x, AT(i+6).x, AT(i+7).x), a_vec);
  const __m256 tmp2 = _mm256_sub_ps (_mm256_set_ps (AT(i  ).y, AT(i+1).y, AT(i+2).y, AT(i+3).y, AT(i+4).y, AT(i+5).y, AT(i+6).y, AT(i+7).y), b_vec);
  return _mm256_add_ps (_mm256_mul_ps (tmp1, tmp1), _mm256_mul_ps (tmp2, tmp2));
}
#endif // ifdef __AVX__

#ifdef __SSE__
// This function computes the squared distances (i.e. the distances without the square root) of 4 points to the center of the circle
template <typename PointT> inline __m128 pcl::SampleConsensusModelCircle2D<PointT>::sqr_dist4 (const std::size_t i, const __m128 a_vec, const __m128 b_vec) const
{
  const __m128 tmp1 = _mm_sub_ps (_mm_set_ps (AT(i  ).x, AT(i+1).x, AT(i+2).x, AT(i+3).x), a_vec);
  const __m128 tmp2 = _mm_sub_ps (_mm_set_ps (AT(i  ).y, AT(i+1).y, AT(i+2).y, AT(i+3).y), b_vec);
  return _mm_add_ps (_mm_mul_ps (tmp1, tmp1), _mm_mul_ps (tmp2, tmp2));
}
#endif // ifdef __SSE__

#undef AT

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle2D<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the circle
  for (std::size_t i = 0; i < indices_->size (); ++i)
    // Calculate the distance from the point to the circle as the difference between
    // dist(point,circle_origin) and circle_radius
    distances[i] = std::abs (std::sqrt (
                                    ( (*input_)[(*indices_)[i]].x - model_coefficients[0] ) *
                                    ( (*input_)[(*indices_)[i]].x - model_coefficients[0] ) +

                                    ( (*input_)[(*indices_)[i]].y - model_coefficients[1] ) *
                                    ( (*input_)[(*indices_)[i]].y - model_coefficients[1] )
                                    ) - model_coefficients[2]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle2D<PointT>::selectWithinDistance (
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
  error_sqr_dists_.clear ();
  inliers.reserve (indices_->size ());
  error_sqr_dists_.reserve (indices_->size ());

  const float sqr_inner_radius = (model_coefficients[2] <= threshold ? 0.0f : (model_coefficients[2] - threshold) * (model_coefficients[2] - threshold));
  const float sqr_outer_radius = (model_coefficients[2] + threshold) * (model_coefficients[2] + threshold);
  // Iterate through the 3d points and calculate the distances from them to the circle
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // To avoid sqrt computation: consider one larger circle (radius + threshold) and one smaller circle (radius - threshold).
    // Valid if point is in larger circle, but not in smaller circle.
    const float sqr_dist = ( (*input_)[(*indices_)[i]].x - model_coefficients[0] ) *
                           ( (*input_)[(*indices_)[i]].x - model_coefficients[0] ) +
                           ( (*input_)[(*indices_)[i]].y - model_coefficients[1] ) *
                           ( (*input_)[(*indices_)[i]].y - model_coefficients[1] );
    if ((sqr_dist <= sqr_outer_radius) && (sqr_dist >= sqr_inner_radius))
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers.push_back ((*indices_)[i]);
      // Only compute exact distance if necessary (if point is inlier)
      error_sqr_dists_.push_back (static_cast<double> (std::abs (std::sqrt (sqr_dist) - model_coefficients[2])));
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelCircle2D<PointT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

#if defined (__AVX__) && defined (__AVX2__)
  return countWithinDistanceAVX (model_coefficients, threshold);
#elif defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
  return countWithinDistanceSSE (model_coefficients, threshold);
#else
  return countWithinDistanceStandard (model_coefficients, threshold);
#endif
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelCircle2D<PointT>::countWithinDistanceStandard (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::size_t i) const
{
  std::size_t nr_p = 0;
  const float sqr_inner_radius = (model_coefficients[2] <= threshold ? 0.0f : (model_coefficients[2] - threshold) * (model_coefficients[2] - threshold));
  const float sqr_outer_radius = (model_coefficients[2] + threshold) * (model_coefficients[2] + threshold);
  // Iterate through the 3d points and calculate the distances from them to the circle
  for (; i < indices_->size (); ++i)
  {
    // To avoid sqrt computation: consider one larger circle (radius + threshold) and one smaller circle (radius - threshold).
    // Valid if point is in larger circle, but not in smaller circle.
    const float sqr_dist = ( (*input_)[(*indices_)[i]].x - model_coefficients[0] ) *
                           ( (*input_)[(*indices_)[i]].x - model_coefficients[0] ) +
                           ( (*input_)[(*indices_)[i]].y - model_coefficients[1] ) *
                           ( (*input_)[(*indices_)[i]].y - model_coefficients[1] );
    if ((sqr_dist <= sqr_outer_radius) && (sqr_dist >= sqr_inner_radius))
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
template <typename PointT> std::size_t
pcl::SampleConsensusModelCircle2D<PointT>::countWithinDistanceSSE (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::size_t i) const
{
  std::size_t nr_p = 0;
  const __m128 a_vec = _mm_set1_ps (model_coefficients[0]);
  const __m128 b_vec = _mm_set1_ps (model_coefficients[1]);
  // To avoid sqrt computation: consider one larger circle (radius + threshold) and one smaller circle (radius - threshold). Valid if point is in larger circle, but not in smaller circle.
  const __m128 sqr_inner_radius = _mm_set1_ps ((model_coefficients[2] <= threshold ? 0.0 : (model_coefficients[2]-threshold)*(model_coefficients[2]-threshold)));
  const __m128 sqr_outer_radius = _mm_set1_ps ((model_coefficients[2]+threshold)*(model_coefficients[2]+threshold));
  __m128i res = _mm_set1_epi32(0); // This corresponds to nr_p: 4 32bit integers that, summed together, hold the number of inliers
  for (; (i + 4) <= indices_->size (); i += 4)
  {
    const __m128 sqr_dist = sqr_dist4 (i, a_vec, b_vec);
    const __m128 mask = _mm_and_ps (_mm_cmplt_ps (sqr_inner_radius, sqr_dist), _mm_cmplt_ps (sqr_dist, sqr_outer_radius)); // The mask contains 1 bits if the corresponding points are inliers, else 0 bits
    res = _mm_add_epi32 (res, _mm_and_si128 (_mm_set1_epi32 (1), _mm_castps_si128 (mask))); // The latter part creates a vector with ones (as 32bit integers) where the points are inliers
    //const int res = _mm_movemask_ps (mask);
    //if (res & 1) nr_p++;
    //if (res & 2) nr_p++;
    //if (res & 4) nr_p++;
    //if (res & 8) nr_p++;
  }
  nr_p += _mm_extract_epi32 (res, 0);
  nr_p += _mm_extract_epi32 (res, 1);
  nr_p += _mm_extract_epi32 (res, 2);
  nr_p += _mm_extract_epi32 (res, 3);

  // Process the remaining points (at most 3)
  nr_p += countWithinDistanceStandard (model_coefficients, threshold, i);
  return (nr_p);
}
#endif

//////////////////////////////////////////////////////////////////////////
#if defined (__AVX__) && defined (__AVX2__)
template <typename PointT> std::size_t
pcl::SampleConsensusModelCircle2D<PointT>::countWithinDistanceAVX (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::size_t i) const
{
  std::size_t nr_p = 0;
  const __m256 a_vec = _mm256_set1_ps (model_coefficients[0]);
  const __m256 b_vec = _mm256_set1_ps (model_coefficients[1]);
  // To avoid sqrt computation: consider one larger circle (radius + threshold) and one smaller circle (radius - threshold). Valid if point is in larger circle, but not in smaller circle.
  const __m256 sqr_inner_radius = _mm256_set1_ps ((model_coefficients[2] <= threshold ? 0.0 : (model_coefficients[2]-threshold)*(model_coefficients[2]-threshold)));
  const __m256 sqr_outer_radius = _mm256_set1_ps ((model_coefficients[2]+threshold)*(model_coefficients[2]+threshold));
  __m256i res = _mm256_set1_epi32(0); // This corresponds to nr_p: 8 32bit integers that, summed together, hold the number of inliers
  for (; (i + 8) <= indices_->size (); i += 8)
  {
    const __m256 sqr_dist = sqr_dist8 (i, a_vec, b_vec);
    const __m256 mask = _mm256_and_ps (_mm256_cmp_ps (sqr_inner_radius, sqr_dist, _CMP_LT_OQ), _mm256_cmp_ps (sqr_dist, sqr_outer_radius, _CMP_LT_OQ)); // The mask contains 1 bits if the corresponding points are inliers, else 0 bits
    res = _mm256_add_epi32 (res, _mm256_and_si256 (_mm256_set1_epi32 (1), _mm256_castps_si256 (mask))); // The latter part creates a vector with ones (as 32bit integers) where the points are inliers
    //const int res = _mm256_movemask_ps (mask);
    //if (res &   1) nr_p++;
    //if (res &   2) nr_p++;
    //if (res &   4) nr_p++;
    //if (res &   8) nr_p++;
    //if (res &  16) nr_p++;
    //if (res &  32) nr_p++;
    //if (res &  64) nr_p++;
    //if (res & 128) nr_p++;
  }
  nr_p += _mm256_extract_epi32 (res, 0);
  nr_p += _mm256_extract_epi32 (res, 1);
  nr_p += _mm256_extract_epi32 (res, 2);
  nr_p += _mm256_extract_epi32 (res, 3);
  nr_p += _mm256_extract_epi32 (res, 4);
  nr_p += _mm256_extract_epi32 (res, 5);
  nr_p += _mm256_extract_epi32 (res, 6);
  nr_p += _mm256_extract_epi32 (res, 7);

  // Process the remaining points (at most 7)
  nr_p += countWithinDistanceStandard (model_coefficients, threshold, i);
  return (nr_p);
}
#endif

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelCircle2D<PointT>::optimizeModelCoefficients (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::optimizeModelCoefficients] Given model is invalid!\n");
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size () <= sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::optimizeModelCoefficients] Not enough inliers to refine/optimize the model's coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
    return;
  }

  OptimizationFunctor functor (this, inliers);
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
      const Indices &inliers, const Eigen::VectorXf &model_coefficients,
      PointCloud &projected_points, bool copy_data_fields) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::projectPoints] Given model is invalid!\n");
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

    // Iterate through the points and project them to the circle
    for (const auto &inlier : inliers)
    {
      float dx = (*input_)[inlier].x - model_coefficients[0];
      float dy = (*input_)[inlier].y - model_coefficients[1];
      float a = std::sqrt ( (model_coefficients[2] * model_coefficients[2]) / (dx * dx + dy * dy) );

      projected_points[inlier].x = a * dx + model_coefficients[0];
      projected_points[inlier].y = a * dy + model_coefficients[1];
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

    // Iterate through the points and project them to the circle
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      float dx = (*input_)[inliers[i]].x - model_coefficients[0];
      float dy = (*input_)[inliers[i]].y - model_coefficients[1];
      float a = std::sqrt ( (model_coefficients[2] * model_coefficients[2]) / (dx * dx + dy * dy) );

      projected_points[i].x = a * dx + model_coefficients[0];
      projected_points[i].y = a * dy + model_coefficients[1];
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelCircle2D<PointT>::doSamplesVerifyModel (
      const std::set<index_t> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCircle2D::doSamplesVerifyModel] Given model is invalid!\n");
    return (false);
  }

  const float sqr_inner_radius = (model_coefficients[2] <= threshold ? 0.0f : (model_coefficients[2] - threshold) * (model_coefficients[2] - threshold));
  const float sqr_outer_radius = (model_coefficients[2] + threshold) * (model_coefficients[2] + threshold);
  for (const auto &index : indices)
  {
    // To avoid sqrt computation: consider one larger circle (radius + threshold) and one smaller circle (radius - threshold).
    // Valid if point is in larger circle, but not in smaller circle.
    const float sqr_dist = ( (*input_)[index].x - model_coefficients[0] ) *
                           ( (*input_)[index].x - model_coefficients[0] ) +
                           ( (*input_)[index].y - model_coefficients[1] ) *
                           ( (*input_)[index].y - model_coefficients[1] );
    if ((sqr_dist > sqr_outer_radius) || (sqr_dist < sqr_inner_radius))
      return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool 
pcl::SampleConsensusModelCircle2D<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
  if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
    return (false);

  if (radius_min_ != -std::numeric_limits<double>::max() && model_coefficients[2] < radius_min_)
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelCircle2D::isModelValid] Radius of circle is too small: should be larger than %g, but is %g.\n",
               radius_min_, model_coefficients[2]);
    return (false);
  }
  if (radius_max_ != std::numeric_limits<double>::max() && model_coefficients[2] > radius_max_)
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelCircle2D::isModelValid] Radius of circle is too big: should be smaller than %g, but is %g.\n",
               radius_max_, model_coefficients[2]);
    return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelCircle2D(T) template class PCL_EXPORTS pcl::SampleConsensusModelCircle2D<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CIRCLE_H_

