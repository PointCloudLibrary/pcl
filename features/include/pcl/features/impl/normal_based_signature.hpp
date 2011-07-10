/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 *  $Id$
 */

#ifndef PCL_FEATURES_IMPL_NORMAL_BASED_SIGNATURE_H_
#define PCL_FEATURES_IMPL_NORMAL_BASED_SIGNATURE_H_

#include "pcl/features/normal_based_signature.h"

template <typename PointT, typename PointNT, typename PointFeature> void
pcl::NormalBasedSignatureEstimation<PointT, PointNT, PointFeature>::computeFeature (FeatureCloud &output)
{
  // do a few checks before starting the computations
  if (!normals_)
  {
    PCL_ERROR ("NormalBasedSignatureEstimation: input normals not set\n");
    return;
  }

  PointFeature test_feature;
  if (N_prime * M_prime != sizeof (test_feature.values) / sizeof (float))
  {
    PCL_ERROR ("NormalBasedSignatureEstimation: not using the proper signature size: %u vs %u\n", N_prime * M_prime, sizeof (test_feature.values) / sizeof (float));
    return;
  }


  tree_->setInputCloud (input_);
  output.points.clear ();
  //for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
  for (size_t index_i = 0; index_i < indices_->size (); ++index_i)
  {
    size_t point_i = indices_->at (index_i);
    Eigen::MatrixXf s_matrix (N, M);
    Eigen::Vector3f center_point = input_->points[point_i].getVector3fMap ();
    for (size_t k = 0; k < N; ++k) {
      Eigen::VectorXf s_row (M);
      for (size_t l = 0; l < M; ++l)
      {
        Eigen::Vector3f normal = normals_->points[point_i].getNormalVector3fMap ();
        Eigen::Vector3f normal_u, normal_v;

        if (fabs (normal.x ()) > 0.0001f)
        {
          normal_u.x () = - normal.y () / normal.x ();
          normal_u.y () = 1.0f;
          normal_u.z () = 0.0f;
          normal_u.normalize ();

        }
        else if (fabs (normal.y ()) > 0.0001f)
        {
          normal_u.x () = 1.0f;
          normal_u.y () = - normal.x () / normal.y ();
          normal_u.z () = 0.0f;
          normal_u.normalize ();
        }
        else
        {
          normal_u.x () = 0.0f;
          normal_u.y () = 1.0f;
          normal_u.z () = - normal.y () / normal.z ();
        }
        normal_v = normal.cross (normal_u);

        Eigen::Vector3f zeta_point = 2.0f*(l+1)*scale_h / M * (cos (2.0f*M_PI*(k+1) / N) * normal_u + sin (2.0f*M_PI*(k+1) / N) * normal_v);

        // compute normal by using the neighbors
        Eigen::Vector3f zeta_point_plus_center = zeta_point + center_point;
        PointT zeta_point_pcl;
        zeta_point_pcl.x = zeta_point_plus_center.x (); zeta_point_pcl.y = zeta_point_plus_center.y (); zeta_point_pcl.z = zeta_point_plus_center.z ();
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        tree_->radiusSearch (zeta_point_pcl,
                             normal_search_radius,
                             k_indices,
                             k_sqr_distances);
        // do k nearest search if there are no neighbors nearby
        if (k_indices.size () == 0)
        {
          k_indices.resize (5);
          k_sqr_distances.resize (5);
          tree_->nearestKSearch (zeta_point_pcl,
                                 5,
                                 k_indices,
                                 k_sqr_distances);
        }
        Eigen::Vector3f average_normal (0.0f, 0.0f, 0.0f);
        float average_normalization_factor = 0.0f;
        // normals weighted by 1/squared_distances
        for (size_t nn_i = 0; nn_i < k_indices.size (); ++nn_i)
        {
          if (k_sqr_distances[nn_i] < 0.0000001f)
          {
            average_normal = normals_->points[k_indices[nn_i]].getNormalVector3fMap ();
            average_normalization_factor = 1.0f;
            break;
          }
          average_normal += normals_->points[k_indices[nn_i]].getNormalVector3fMap () / k_sqr_distances[nn_i];
          average_normalization_factor += 1.0f / k_sqr_distances[nn_i];
        }
        average_normal /= average_normalization_factor;
        float s = zeta_point.dot (average_normal) / zeta_point.norm ();
        s_row[l] = s;
      }


      // do DCT on the s_matrix row-wise
      Eigen::VectorXf dct_row (M);
      for (size_t k = 0; k < s_row.size (); ++k)
      {
        float Xk = 0.0f;
        for (size_t n = 0; n < s_row.size (); ++n)
          Xk += s_row[n] * cos (M_PI / M * (n + 0.5f) * k);
        dct_row[k] = Xk;
      }
      s_row = dct_row;
      s_matrix.row (k) = dct_row;
    }

    // do DFT on the s_matrix column-wise
    Eigen::MatrixXf dft_matrix (N, M);
    for (size_t column_i = 0; column_i < M; ++column_i)
    {
      Eigen::VectorXf dft_col (N);
      for (size_t k = 0; k < N; ++k)
      {
        float Xk_real = 0.0f, Xk_imag = 0.0f;
        for (size_t n = 0; n < N; ++n)
        {
          Xk_real += s_matrix(n, column_i) * cos (2.0f * M_PI / N * k * n);
          Xk_imag += s_matrix(n, column_i) * sin (2.0f * M_PI / N * k * n);
        }
        dft_col[k] = sqrt (Xk_real*Xk_real + Xk_imag*Xk_imag);
      }
      dft_matrix.col (column_i) = dft_col;
    }

    Eigen::MatrixXf final_matrix = dft_matrix.block (0, 0, N_prime, M_prime);

    PointFeature feature_point;
    float feature[N_prime * M_prime];
    for (size_t i = 0; i < N_prime; ++i)
      for (size_t j = 0; j < M_prime; ++j)
        feature_point.values[i*M_prime + j] = final_matrix (i, j);
    output.points.push_back (feature_point);
  }
}



#define PCL_INSTANTIATE_NormalBasedSignatureEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::NormalBasedSignatureEstimation<T,NT,OutT>;


#endif /* PCL_FEATURES_IMPL_NORMAL_BASED_SIGNATURE_H_ */
