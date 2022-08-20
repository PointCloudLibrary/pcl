/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
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
 *  $Id$
 */

#ifndef PCL_FEATURES_IMPL_NORMAL_BASED_SIGNATURE_H_
#define PCL_FEATURES_IMPL_NORMAL_BASED_SIGNATURE_H_

#include <pcl/features/normal_based_signature.h>

template <typename PointT, typename PointNT, typename PointFeature> void
pcl::NormalBasedSignatureEstimation<PointT, PointNT, PointFeature>::computeFeature (FeatureCloud &output)
{
  // do a few checks before starting the computations

  PointFeature test_feature;
  if (N_prime_ * M_prime_ != sizeof (test_feature.values) / sizeof (float))
  {
    PCL_ERROR ("NormalBasedSignatureEstimation: not using the proper signature size: %u vs %u\n", N_prime_ * M_prime_, sizeof (test_feature.values) / sizeof (float));
    return;
  }

  pcl::Indices k_indices;
  std::vector<float> k_sqr_distances;

  tree_->setInputCloud (input_);
  output.resize (indices_->size ());

  for (std::size_t index_i = 0; index_i < indices_->size (); ++index_i)
  {
    std::size_t point_i = (*indices_)[index_i];
    Eigen::MatrixXf s_matrix (N_, M_);

    Eigen::Vector4f center_point = (*input_)[point_i].getVector4fMap ();

    for (std::size_t k = 0; k < N_; ++k)
    {
      Eigen::VectorXf s_row (M_);

      for (std::size_t l = 0; l < M_; ++l)
      {
        Eigen::Vector4f normal = (*normals_)[point_i].getNormalVector4fMap ();
        Eigen::Vector4f normal_u = Eigen::Vector4f::Zero ();
        Eigen::Vector4f normal_v = Eigen::Vector4f::Zero ();

        if (std::abs (normal.x ()) > 0.0001f)
        {
          normal_u.x () = - normal.y () / normal.x ();
          normal_u.y () = 1.0f;
          normal_u.z () = 0.0f;
          normal_u.normalize ();

        }
        else if (std::abs (normal.y ()) > 0.0001f)
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
        normal_v = normal.cross3 (normal_u);

        Eigen::Vector4f zeta_point = 2.0f * static_cast<float> (l + 1) * scale_h_ / static_cast<float> (M_) * 
            (std::cos (2.0f * static_cast<float> (M_PI) * static_cast<float> ((k + 1) / N_)) * normal_u + 
             sinf (2.0f * static_cast<float> (M_PI) * static_cast<float> ((k + 1) / N_)) * normal_v);

        // Compute normal by using the neighbors
        Eigen::Vector4f zeta_point_plus_center = zeta_point + center_point;
        PointT zeta_point_pcl;
        zeta_point_pcl.x = zeta_point_plus_center.x (); zeta_point_pcl.y = zeta_point_plus_center.y (); zeta_point_pcl.z = zeta_point_plus_center.z ();

        tree_->radiusSearch (zeta_point_pcl, search_radius_, k_indices, k_sqr_distances);

        // Do k nearest search if there are no neighbors nearby
        if (k_indices.empty ())
        {
          k_indices.resize (5);
          k_sqr_distances.resize (5);
          tree_->nearestKSearch (zeta_point_pcl, 5, k_indices, k_sqr_distances);
        }
        
        Eigen::Vector4f average_normal = Eigen::Vector4f::Zero ();

        float average_normalization_factor = 0.0f;

        // Normals weighted by 1/squared_distances
        for (std::size_t nn_i = 0; nn_i < k_indices.size (); ++nn_i)
        {
          if (k_sqr_distances[nn_i] < 1e-7f)
          {
            average_normal = (*normals_)[k_indices[nn_i]].getNormalVector4fMap ();
            average_normalization_factor = 1.0f;
            break;
          }
          average_normal += (*normals_)[k_indices[nn_i]].getNormalVector4fMap () / k_sqr_distances[nn_i];
          average_normalization_factor += 1.0f / k_sqr_distances[nn_i];
        }
        average_normal /= average_normalization_factor;
        float s = zeta_point.dot (average_normal) / zeta_point.norm ();
        s_row[l] = s;
      }

      // do DCT on the s_matrix row-wise
      Eigen::VectorXf dct_row (M_);
      for (Eigen::Index m = 0; m < s_row.size (); ++m)
      {
        float Xk = 0.0f;
        for (Eigen::Index n = 0; n < s_row.size (); ++n)
          Xk += static_cast<float> (s_row[n] * std::cos (M_PI / (static_cast<double> (M_ * n) + 0.5) * static_cast<double> (k)));
        dct_row[m] = Xk;
      }
      s_row = dct_row;
      s_matrix.row (k).matrix () = dct_row;
    }

    // do DFT on the s_matrix column-wise
    Eigen::MatrixXf dft_matrix (N_, M_);
    for (std::size_t column_i = 0; column_i < M_; ++column_i)
    {
      Eigen::VectorXf dft_col (N_);
      for (std::size_t k = 0; k < N_; ++k)
      {
        float Xk_real = 0.0f, Xk_imag = 0.0f;
        for (std::size_t n = 0; n < N_; ++n)
        {
          Xk_real += static_cast<float> (s_matrix (n, column_i) * std::cos (2.0f * M_PI / static_cast<double> (N_ * k * n)));
          Xk_imag += static_cast<float> (s_matrix (n, column_i) * sin (2.0f * M_PI / static_cast<double> (N_ * k * n)));
        }
        dft_col[k] = std::sqrt (Xk_real*Xk_real + Xk_imag*Xk_imag);
      }
      dft_matrix.col (column_i).matrix () = dft_col;
    }

    Eigen::MatrixXf final_matrix = dft_matrix.block (0, 0, N_prime_, M_prime_);

    PointFeature feature_point;
    for (std::size_t i = 0; i < N_prime_; ++i)
      for (std::size_t j = 0; j < M_prime_; ++j)
        feature_point.values[i*M_prime_ + j] = final_matrix (i, j);

    output[index_i] = feature_point;
  }
}



#define PCL_INSTANTIATE_NormalBasedSignatureEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::NormalBasedSignatureEstimation<T,NT,OutT>;


#endif /* PCL_FEATURES_IMPL_NORMAL_BASED_SIGNATURE_H_ */
