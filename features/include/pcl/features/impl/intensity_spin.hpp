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

#ifndef PCL_FEATURES_IMPL_INTENSITY_SPIN_H_
#define PCL_FEATURES_IMPL_INTENSITY_SPIN_H_

#include <pcl/features/intensity_spin.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntensitySpinEstimation<PointInT, PointOutT>::computeIntensitySpinImage (
      const PointCloudIn &cloud, float radius, float sigma, 
      int k,
      const std::vector<int> &indices, 
      const std::vector<float> &squared_distances, 
      Eigen::MatrixXf &intensity_spin_image)
{
  // Determine the number of bins to use based on the size of intensity_spin_image
  int nr_distance_bins = static_cast<int> (intensity_spin_image.cols ());
  int nr_intensity_bins = static_cast<int> (intensity_spin_image.rows ());

  // Find the min and max intensity values in the given neighborhood
  float min_intensity = std::numeric_limits<float>::max ();
  float max_intensity = -std::numeric_limits<float>::max ();
  for (int idx = 0; idx < k; ++idx)
  {
    min_intensity = (std::min) (min_intensity, cloud.points[indices[idx]].intensity);
    max_intensity = (std::max) (max_intensity, cloud.points[indices[idx]].intensity);
  }

  float constant = 1.0f / (2.0f * sigma_ * sigma_);
  // Compute the intensity spin image
  intensity_spin_image.setZero ();
  for (int idx = 0; idx < k; ++idx)
  {
    // Normalize distance and intensity values to: 0.0 <= d,i < nr_distance_bins,nr_intensity_bins
    const float eps = std::numeric_limits<float>::epsilon ();
    float d = static_cast<float> (nr_distance_bins) * std::sqrt (squared_distances[idx]) / (radius + eps);
    float i = static_cast<float> (nr_intensity_bins) * 
              (cloud.points[indices[idx]].intensity - min_intensity) / (max_intensity - min_intensity + eps);

    if (sigma == 0)
    {
      // If sigma is zero, update the histogram with no smoothing kernel
      int d_idx = static_cast<int> (d);
      int i_idx = static_cast<int> (i);
      intensity_spin_image (i_idx, d_idx) += 1;
    }
    else
    {
      // Compute the bin indices that need to be updated (+/- 3 standard deviations)
      int d_idx_min = (std::max)(static_cast<int> (floor (d - 3*sigma)), 0);
      int d_idx_max = (std::min)(static_cast<int> (ceil  (d + 3*sigma)), nr_distance_bins - 1);
      int i_idx_min = (std::max)(static_cast<int> (floor (i - 3*sigma)), 0);
      int i_idx_max = (std::min)(static_cast<int> (ceil  (i + 3*sigma)), nr_intensity_bins - 1);
   
      // Update the appropriate bins of the histogram 
      for (int i_idx = i_idx_min; i_idx <= i_idx_max; ++i_idx)  
      {
        for (int d_idx = d_idx_min; d_idx <= d_idx_max; ++d_idx)
        {
          // Compute a "soft" update weight based on the distance between the point and the bin
          float w = expf (-powf (d - static_cast<float> (d_idx), 2.0f) * constant - powf (i - static_cast<float> (i_idx), 2.0f) * constant);
          intensity_spin_image (i_idx, d_idx) += w;
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntensitySpinEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Make sure a search radius is set
  if (search_radius_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] The search radius must be set before computing the feature!\n",
               getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Make sure the spin image has valid dimensions
  if (nr_intensity_bins_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] The number of intensity bins must be greater than zero!\n",
               getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (nr_distance_bins_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] The number of distance bins must be greater than zero!\n",
               getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  Eigen::MatrixXf intensity_spin_image (nr_intensity_bins_, nr_distance_bins_);
  // Allocate enough space to hold the radiusSearch results
  std::vector<int> nn_indices (surface_->points.size ());
  std::vector<float> nn_dist_sqr (surface_->points.size ());
 
  output.is_dense = true;
  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    // Find neighbors within the search radius
    // TODO: do we want to use searchForNeigbors instead?
    int k = tree_->radiusSearch ((*indices_)[idx], search_radius_, nn_indices, nn_dist_sqr);
    if (k == 0)
    {
      for (int bin = 0; bin < nr_intensity_bins_ * nr_distance_bins_; ++bin)
        output.points[idx].histogram[bin] = std::numeric_limits<float>::quiet_NaN ();
      output.is_dense = false;
      continue;
    }

    // Compute the intensity spin image
    computeIntensitySpinImage (*surface_, static_cast<float> (search_radius_), sigma_, k, nn_indices, nn_dist_sqr, intensity_spin_image);

    // Copy into the resultant cloud
    int bin = 0;
    for (int bin_j = 0; bin_j < intensity_spin_image.cols (); ++bin_j)
      for (int bin_i = 0; bin_i < intensity_spin_image.rows (); ++bin_i)
        output.points[idx].histogram[bin++] = intensity_spin_image (bin_i, bin_j);
  }
}

#define PCL_INSTANTIATE_IntensitySpinEstimation(T,NT) template class PCL_EXPORTS pcl::IntensitySpinEstimation<T,NT>;

#endif    // PCL_FEATURES_IMPL_INTENSITY_SPIN_H_ 

