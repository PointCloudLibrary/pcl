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

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelPlane<PointT>::isSampleGood (const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }

  // Check if the sample points are collinear
  // Similar checks are implemented as precaution in computeModelCoefficients,
  // so if you find the need to fix something in here, look there, too.
  pcl::Vector3fMapConst p0 = (*input_)[samples[0]].getVector3fMap ();
  pcl::Vector3fMapConst p1 = (*input_)[samples[1]].getVector3fMap ();
  pcl::Vector3fMapConst p2 = (*input_)[samples[2]].getVector3fMap ();

  // Check if the norm of the cross-product would be non-zero, otherwise
  // normalization will fail. One could also interpret this as kind of check
  // if the triangle spanned by those three points would have an area greater
  // than zero.
  if ((p1 - p0).cross(p2 - p0).stableNorm() < Eigen::NumTraits<float>::dummy_precision ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::isSampleGood] Sample points too similar or collinear!\n");
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelPlane<PointT>::computeModelCoefficients (
      const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  // The checks are redundant with isSampleGood above, but since most of the
  // computed values are also used to compute the model coefficients, this might
  // be a situation where this duplication is acceptable.
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  pcl::Vector3fMapConst p0 = (*input_)[samples[0]].getVector3fMap ();
  pcl::Vector3fMapConst p1 = (*input_)[samples[1]].getVector3fMap ();
  pcl::Vector3fMapConst p2 = (*input_)[samples[2]].getVector3fMap ();

  const Eigen::Vector3f cross = (p1 - p0).cross(p2 - p0);
  const float crossNorm = cross.stableNorm();

  // Checking for collinearity here
  if (crossNorm < Eigen::NumTraits<float>::dummy_precision ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::computeModelCoefficients] Chosen samples are collinear!\n");
    return (false);
  }

  // Compute the plane coefficients from the 3 given points in a straightforward manner
  // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
  model_coefficients.resize (model_size_);
  model_coefficients.template head<3>() = cross / crossNorm;

  // ... + d = 0
  model_coefficients[3] = -1.0f * (model_coefficients.template head<3>().dot (p0));

  PCL_DEBUG ("[pcl::SampleConsensusModelPlane::computeModelCoefficients] Model is (%g,%g,%g,%g).\n",
             model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3]);
  return (true);
}

#define AT(POS) ((*input_)[(*indices_)[(POS)]])

#ifdef __AVX__
// This function computes the distances of 8 points to the plane
template <typename PointT> inline __m256 pcl::SampleConsensusModelPlane<PointT>::dist8 (const std::size_t i, const __m256 &a_vec, const __m256 &b_vec, const __m256 &c_vec, const __m256 &d_vec, const __m256 &abs_help) const
{
  // The andnot-function realizes an abs-operation: the sign bit is removed
  return _mm256_andnot_ps (abs_help,
        _mm256_add_ps (_mm256_add_ps (_mm256_mul_ps (a_vec, _mm256_set_ps (AT(i  ).x, AT(i+1).x, AT(i+2).x, AT(i+3).x, AT(i+4).x, AT(i+5).x, AT(i+6).x, AT(i+7).x)),
                                      _mm256_mul_ps (b_vec, _mm256_set_ps (AT(i  ).y, AT(i+1).y, AT(i+2).y, AT(i+3).y, AT(i+4).y, AT(i+5).y, AT(i+6).y, AT(i+7).y))),
                       _mm256_add_ps (_mm256_mul_ps (c_vec, _mm256_set_ps (AT(i  ).z, AT(i+1).z, AT(i+2).z, AT(i+3).z, AT(i+4).z, AT(i+5).z, AT(i+6).z, AT(i+7).z)),
                                      d_vec))); // TODO this could be replaced by three fmadd-instructions (if available), but the speed gain would probably be minimal
}
#endif // ifdef __AVX__

#ifdef __SSE__
// This function computes the distances of 4 points to the plane
template <typename PointT> inline __m128 pcl::SampleConsensusModelPlane<PointT>::dist4 (const std::size_t i, const __m128 &a_vec, const __m128 &b_vec, const __m128 &c_vec, const __m128 &d_vec, const __m128 &abs_help) const
{
  // The andnot-function realizes an abs-operation: the sign bit is removed
  return _mm_andnot_ps (abs_help,
        _mm_add_ps (_mm_add_ps (_mm_mul_ps (a_vec, _mm_set_ps (AT(i  ).x, AT(i+1).x, AT(i+2).x, AT(i+3).x)),
                                _mm_mul_ps (b_vec, _mm_set_ps (AT(i  ).y, AT(i+1).y, AT(i+2).y, AT(i+3).y))),
                    _mm_add_ps (_mm_mul_ps (c_vec, _mm_set_ps (AT(i  ).z, AT(i+1).z, AT(i+2).z, AT(i+3).z)),
                                d_vec))); // TODO this could be replaced by three fmadd-instructions (if available), but the speed gain would probably be minimal
}
#endif // ifdef __SSE__

#undef AT

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
    /*distances[i] = std::abs (model_coefficients[0] * (*input_)[(*indices_)[i]].x +
                         model_coefficients[1] * (*input_)[(*indices_)[i]].y +
                         model_coefficients[2] * (*input_)[(*indices_)[i]].z +
                         model_coefficients[3]);*/
    Eigen::Vector4f pt ((*input_)[(*indices_)[i]].x,
                        (*input_)[(*indices_)[i]].y,
                        (*input_)[(*indices_)[i]].z,
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
    Eigen::Vector4f pt ((*input_)[(*indices_)[i]].x,
                        (*input_)[(*indices_)[i]].y,
                        (*input_)[(*indices_)[i]].z,
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
pcl::SampleConsensusModelPlane<PointT>::countWithinDistanceStandard (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::size_t i) const
{
  std::size_t nr_p = 0;
  // Iterate through the 3d points and calculate the distances from them to the plane
  for (; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the plane normal as the dot product
    // D = (P-A).N/|N|
    Eigen::Vector4f pt ((*input_)[(*indices_)[i]].x,
                        (*input_)[(*indices_)[i]].y,
                        (*input_)[(*indices_)[i]].z,
                        1.0f);
    if (std::abs (model_coefficients.dot (pt)) < threshold)
    {
      nr_p++;
    }
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
#if defined (__SSE__) && defined (__SSE2__) && defined (__SSE4_1__)
template <typename PointT> std::size_t
pcl::SampleConsensusModelPlane<PointT>::countWithinDistanceSSE (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::size_t i) const
{
  std::size_t nr_p = 0;
  const __m128 a_vec = _mm_set1_ps (model_coefficients[0]);
  const __m128 b_vec = _mm_set1_ps (model_coefficients[1]);
  const __m128 c_vec = _mm_set1_ps (model_coefficients[2]);
  const __m128 d_vec = _mm_set1_ps (model_coefficients[3]);
  const __m128 threshold_vec = _mm_set1_ps (threshold);
  const __m128 abs_help = _mm_set1_ps (-0.0F); // -0.0F (negative zero) means that all bits are 0, only the sign bit is 1
  __m128i res = _mm_set1_epi32(0); // This corresponds to nr_p: 4 32bit integers that, summed together, hold the number of inliers
  for (; (i + 4) <= indices_->size (); i += 4)
  {
    const __m128 mask = _mm_cmplt_ps (dist4 (i, a_vec, b_vec, c_vec, d_vec, abs_help), threshold_vec); // The mask contains 1 bits if the corresponding points are inliers, else 0 bits
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
  nr_p += countWithinDistanceStandard(model_coefficients, threshold, i);
  return (nr_p);
}
#endif

//////////////////////////////////////////////////////////////////////////
#if defined (__AVX__) && defined (__AVX2__)
template <typename PointT> std::size_t
pcl::SampleConsensusModelPlane<PointT>::countWithinDistanceAVX (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::size_t i) const
{
  std::size_t nr_p = 0;
  const __m256 a_vec = _mm256_set1_ps (model_coefficients[0]);
  const __m256 b_vec = _mm256_set1_ps (model_coefficients[1]);
  const __m256 c_vec = _mm256_set1_ps (model_coefficients[2]);
  const __m256 d_vec = _mm256_set1_ps (model_coefficients[3]);
  const __m256 threshold_vec = _mm256_set1_ps (threshold);
  const __m256 abs_help = _mm256_set1_ps (-0.0F); // -0.0F (negative zero) means that all bits are 0, only the sign bit is 1
  __m256i res = _mm256_set1_epi32(0); // This corresponds to nr_p: 8 32bit integers that, summed together, hold the number of inliers
  for (; (i + 8) <= indices_->size (); i += 8)
  {
    const __m256 mask = _mm256_cmp_ps (dist8 (i, a_vec, b_vec, c_vec, d_vec, abs_help), threshold_vec, _CMP_LT_OQ); // The mask contains 1 bits if the corresponding points are inliers, else 0 bits
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
  nr_p += countWithinDistanceStandard(model_coefficients, threshold, i);
  return (nr_p);
}
#endif

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

  if (0 == computeMeanAndCovarianceMatrix (*input_, inliers, covariance_matrix, xyz_centroid))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPlane::optimizeModelCoefficients] computeMeanAndCovarianceMatrix failed (returned 0) because there are no valid inliers.\n");
    optimized_coefficients = model_coefficients;
    return;
  }

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
    projected_points.resize (input_->size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < input_->size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[i], projected_points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (const auto &inlier : inliers)
    {
      // Calculate the distance from the point to the plane
      Eigen::Vector4f p ((*input_)[inlier].x,
                         (*input_)[inlier].y,
                         (*input_)[inlier].z,
                         1);
      // use normalized coefficients to calculate the scalar projection
      float distance_to_plane = tmp_mc.dot (p);

      pcl::Vector4fMap pp = projected_points[inlier].getVector4fMap ();
      pp.matrix () = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
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
    {
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[inliers[i]], projected_points[i]));
    }

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      // Calculate the distance from the point to the plane
      Eigen::Vector4f p ((*input_)[inliers[i]].x,
                         (*input_)[inliers[i]].y,
                         (*input_)[inliers[i]].z,
                         1.0f);
      // use normalized coefficients to calculate the scalar projection
      float distance_to_plane = tmp_mc.dot (p);

      pcl::Vector4fMap pp = projected_points[i].getVector4fMap ();
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
    Eigen::Vector4f pt ((*input_)[index].x,
                        (*input_)[index].y,
                        (*input_)[index].z,
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

