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

#ifndef PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_
#define PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_

#include "pcl/features/statistical_multiscale_interest_region_extraction.h"


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::generateCloudGraph ()
{

}



//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::computeRegionsOfInterest ()
{
  if (!initCompute ())
  {
    PCL_ERROR ("StatisticalMultiscaleInterestRegionExtraction: not completely initialized\n");
    return;
  }

  for (std::vector<float>::iterator scale_it = scale_values.begin (); scale_it != scale_values.end (); ++scale_it)
  {
    float scale_squared = (*scale_it) * (*scale_it);

    // calculate point density for each point x_i
    float point_density[input_->points.size ()];
    float phi[input_->points.size ()][input_->points.size ()];
    float F[input_->points.size ()];
    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
    {
      float point_density_i = 0.0;
      for (size_t point_j = 0; point_j < input_->points.size (); ++point_j)
      {
        float d_g = geodesic_distances[point_i][point_j];
        float phi_i_j = 1.0 / sqrt(2.0*M_PI*scale_squared) * exp( (-1) * d_g*d_g / (2.0*scale_squared));

        point_density_i += phi_i_j;
        phi[point_i][point_j] = phi_i_j;
      }
      point_density[point_i] = point_density_i;
    }

    // compute weights for each pair (x_i, x_j), evaluate the operator A_hat
    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
    {
      float A_hat_normalization = 0.0;
      PointT A_hat; A_hat.x = A_hat.y = A_hat.z = 0.0;
      for (size_t point_j = 0; point_j < input_->points.size (); ++point_j)
      {
        float phi_hat_i_j = phi[point_i][point_j] / (point_density[point_i] * point_density[point_j]);
        A_hat_normalization += phi_hat_i_j;

        PointT aux = input_->points[point_j];
        aux.x *= phi_hat_i_j; aux.y *= phi_hat_i_j; aux.z *= phi_hat_i_j;

        A_hat.x += aux.x; A_hat.y += aux.y; A_hat.z += aux.z;
      }
      A_hat.x /= A_hat_normalization; A_hat.y /= A_hat_normalization; A_hat.z /= A_hat_normalization;

      // compute the invariant F
      float aux = 2.0 / (*scale_it) * euclideanDistance<PointT, PointT> (A_hat, input_->points[point_i]);
      F[point_i] = aux * exp (-aux);
    }

  }
}


#define PCL_INSTANTIATE_StatisticalMultiscaleInterestRegionExtraction(T) template class PCL_EXPORTS pcl::StatisticalMultiscaleInterestRegionExtraction<T>;

#endif /* PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_ */

