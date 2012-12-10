/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_IMPL_INTENSITY_GRADIENT_H_
#define PCL_FEATURES_IMPL_INTENSITY_GRADIENT_H_

#include <pcl/features/intensity_gradient.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename IntensitySelectorT> void
pcl::IntensityGradientEstimation <PointInT, PointNT, PointOutT, IntensitySelectorT>::computePointIntensityGradient (
  const pcl::PointCloud <PointInT> &cloud, const std::vector <int> &indices,
  const Eigen::Vector3f &point, float mean_intensity, const Eigen::Vector3f &normal, Eigen::Vector3f &gradient)
{
  if (indices.size () < 3)
  {
    gradient[0] = gradient[1] = gradient[2] = std::numeric_limits<float>::quiet_NaN ();
    return;
  }

  Eigen::Matrix3f A = Eigen::Matrix3f::Zero ();
  Eigen::Vector3f b = Eigen::Vector3f::Zero ();

  for (size_t i_point = 0; i_point < indices.size (); ++i_point)
  {
    PointInT p = cloud.points[indices[i_point]];
    if (!pcl_isfinite (p.x) ||
        !pcl_isfinite (p.y) ||
        !pcl_isfinite (p.z) ||
        !pcl_isfinite (intensity_ (p)))
      continue;

    p.x -= point[0];
    p.y -= point[1];
    p.z -= point[2];
    intensity_.demean (p, mean_intensity);

    A (0, 0) += p.x * p.x;
    A (0, 1) += p.x * p.y;
    A (0, 2) += p.x * p.z;

    A (1, 1) += p.y * p.y;
    A (1, 2) += p.y * p.z;

    A (2, 2) += p.z * p.z;

    b[0] += p.x * intensity_ (p);
    b[1] += p.y * intensity_ (p);
    b[2] += p.z * intensity_ (p);
  }
  // Fill in the lower triangle of A
  A (1, 0) = A (0, 1);
  A (2, 0) = A (0, 2);
  A (2, 1) = A (1, 2);

//*
  Eigen::Vector3f x = A.colPivHouseholderQr ().solve (b);
/*/

  Eigen::Vector3f eigen_values;
  Eigen::Matrix3f eigen_vectors;
  eigen33 (A, eigen_vectors, eigen_values);

  b = eigen_vectors.transpose () * b;

  if ( eigen_values (0) != 0)
    b (0) /= eigen_values (0);
  else
    b (0) = 0;

  if ( eigen_values (1) != 0)
    b (1) /= eigen_values (1);
  else
    b (1) = 0;

  if ( eigen_values (2) != 0)
    b (2) /= eigen_values (2);
  else
    b (2) = 0;


  Eigen::Vector3f x = eigen_vectors * b;

//  if (A.col (0).squaredNorm () != 0)
//    x [0] /= A.col (0).squaredNorm ();
//  b -= x [0] * A.col (0);
//
//
//  if (A.col (1).squaredNorm ()  != 0)
//    x [1] /= A.col (1).squaredNorm ();
//  b -= x[1] * A.col (1);
//
//  x [2] = b.dot (A.col (2));
//  if (A.col (2).squaredNorm () != 0)
//    x[2] /= A.col (2).squaredNorm ();
  // Fit a hyperplane to the data

//*/
//  std::cout << A << "\n*\n" << bb << "\n=\n" << x << "\nvs.\n" << x2 << "\n\n";
//  std::cout << A * x << "\nvs.\n" << A * x2 << "\n\n------\n";
  // Project the gradient vector, x, onto the tangent plane
  gradient = (Eigen::Matrix3f::Identity () - normal*normal.transpose ()) * x;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT, typename IntensitySelectorT> void
pcl::IntensityGradientEstimation<PointInT, PointNT, PointOutT, IntensitySelectorT>::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);
  output.is_dense = true;

  // If the data is dense, we don't need to check for NaN
  if (surface_->is_dense)
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      PointOutT &p_out = output.points[idx];

      if (!this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists))
      {
        p_out.gradient[0] = p_out.gradient[1] = p_out.gradient[2] = std::numeric_limits<float>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }

      Eigen::Vector3f centroid;
      float mean_intensity = 0;
      // Initialize to 0
      centroid.setZero ();
      for (size_t i = 0; i < nn_indices.size (); ++i)
      {
        centroid += surface_->points[nn_indices[i]].getVector3fMap ();
        mean_intensity += intensity_ (surface_->points[nn_indices[i]]);
      }
      centroid /= static_cast<float> (nn_indices.size ());
      mean_intensity /= static_cast<float> (nn_indices.size ());

      Eigen::Vector3f normal = Eigen::Vector3f::Map (normals_->points[(*indices_) [idx]].normal);
      Eigen::Vector3f gradient;
      computePointIntensityGradient (*surface_, nn_indices, centroid, mean_intensity, normal, gradient);

      p_out.gradient[0] = gradient[0];
      p_out.gradient[1] = gradient[1];
      p_out.gradient[2] = gradient[2];
    }
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
    // Iterating over the entire index vector
    for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
    {
      PointOutT &p_out = output.points[idx];
      if (!isFinite ((*surface_) [(*indices_)[idx]]) ||
          !this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists))
      {
        p_out.gradient[0] = p_out.gradient[1] = p_out.gradient[2] = std::numeric_limits<float>::quiet_NaN ();
        output.is_dense = false;
        continue;
      }
      Eigen::Vector3f centroid;
      float mean_intensity = 0;
      // Initialize to 0
      centroid.setZero ();
      unsigned cp = 0;
      for (size_t i = 0; i < nn_indices.size (); ++i)
      {
        // Check if the point is invalid
        if (!isFinite ((*surface_) [nn_indices[i]]))
          continue;

        centroid += surface_->points [nn_indices[i]].getVector3fMap ();
        mean_intensity += intensity_ (surface_->points [nn_indices[i]]);
        ++cp;
      }
      centroid /= static_cast<float> (cp);
      mean_intensity /= static_cast<float> (cp);
      Eigen::Vector3f normal = Eigen::Vector3f::Map (normals_->points[(*indices_) [idx]].normal);
      Eigen::Vector3f gradient;
      computePointIntensityGradient (*surface_, nn_indices, centroid, mean_intensity, normal, gradient);

      p_out.gradient[0] = gradient[0];
      p_out.gradient[1] = gradient[1];
      p_out.gradient[2] = gradient[2];
    }
  }
}

#define PCL_INSTANTIATE_IntensityGradientEstimation(InT,NT,OutT) template class PCL_EXPORTS pcl::IntensityGradientEstimation<InT,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_INTENSITY_GRADIENT_H_
