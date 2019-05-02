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
 *  $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_USC_HPP_
#define PCL_FEATURES_IMPL_USC_HPP_

#include <pcl/features/usc.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/common/geometry.h>
#include <pcl/common/angles.h>
#include <pcl/common/utils.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename PointRFT> bool
pcl::UniqueShapeContext<PointInT, PointOutT, PointRFT>::initCompute ()
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // Default LRF estimation alg: SHOTLocalReferenceFrameEstimation
  typename SHOTLocalReferenceFrameEstimation<PointInT, PointRFT>::Ptr lrf_estimator(new SHOTLocalReferenceFrameEstimation<PointInT, PointRFT>());
  lrf_estimator->setRadiusSearch (local_radius_);
  lrf_estimator->setInputCloud (input_);
  lrf_estimator->setIndices (indices_);
  if (!fake_surface_)
    lrf_estimator->setSearchSurface(surface_);

  if (!FeatureWithLocalReferenceFrames<PointInT, PointRFT>::initLocalReferenceFrames (indices_->size (), lrf_estimator))
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  if (search_radius_< min_radius_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] search_radius_ must be GREATER than min_radius_.\n", getClassName ().c_str ());
    return (false);
  }

  // Update descriptor length
  descriptor_length_ = elevation_bins_ * azimuth_bins_ * radius_bins_;

  // Compute radial, elevation and azimuth divisions
  float azimuth_interval = 360.0f / static_cast<float> (azimuth_bins_);
  float elevation_interval = 180.0f / static_cast<float> (elevation_bins_);

  // Reallocate divisions and volume lut
  radii_interval_.clear ();
  phi_divisions_.clear ();
  theta_divisions_.clear ();
  volume_lut_.clear ();

  // Fills radii interval based on formula (1) in section 2.1 of Frome's paper
  radii_interval_.resize (radius_bins_ + 1);
  for (size_t j = 0; j < radius_bins_ + 1; j++)
    radii_interval_[j] = static_cast<float> (exp (log (min_radius_) + ((static_cast<float> (j) / static_cast<float> (radius_bins_)) * log (search_radius_/min_radius_))));

  // Fill theta didvisions of elevation
  theta_divisions_.resize (elevation_bins_+1);
  for (size_t k = 0; k < elevation_bins_+1; k++)
    theta_divisions_[k] = static_cast<float> (k) * elevation_interval;

  // Fill phi didvisions of elevation
  phi_divisions_.resize (azimuth_bins_+1);
  for (size_t l = 0; l < azimuth_bins_+1; l++)
    phi_divisions_[l] = static_cast<float> (l) * azimuth_interval;

  // LookUp Table that contains the volume of all the bins
  // "phi" term of the volume integral
  // "integr_phi" has always the same value so we compute it only one time
  float integr_phi  = pcl::deg2rad (phi_divisions_[1]) - pcl::deg2rad (phi_divisions_[0]);
  // exponential to compute the cube root using pow
  float e = 1.0f / 3.0f;
  // Resize volume look up table
  volume_lut_.resize (radius_bins_ * elevation_bins_ * azimuth_bins_);
  // Fill volumes look up table
  for (size_t j = 0; j < radius_bins_; j++)
  {
    // "r" term of the volume integral
    float integr_r = (radii_interval_[j+1]*radii_interval_[j+1]*radii_interval_[j+1] / 3) - (radii_interval_[j]*radii_interval_[j]*radii_interval_[j]/ 3);

    for (size_t k = 0; k < elevation_bins_; k++)
    {
      // "theta" term of the volume integral
      float integr_theta = cosf (deg2rad (theta_divisions_[k])) - cosf (deg2rad (theta_divisions_[k+1]));
      // Volume
      float V = integr_phi * integr_theta * integr_r;
      // Compute cube root of the computed volume commented for performance but left
      // here for clarity
      // float cbrt = pow(V, e);
      // cbrt = 1 / cbrt;

      for (size_t l = 0; l < azimuth_bins_; l++)
        // Store in lut 1/cbrt
        //volume_lut_[ (l*elevation_bins_*radius_bins_) + k*radius_bins_ + j ] = cbrt;
        volume_lut_[(l*elevation_bins_*radius_bins_) + k*radius_bins_ + j] = 1.0f / powf (V, e);
    }
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename PointRFT> void
pcl::UniqueShapeContext<PointInT, PointOutT, PointRFT>::computePointDescriptor (size_t index, /*float rf[9],*/ std::vector<float> &desc)
{
  pcl::Vector3fMapConst origin = input_->points[(*indices_)[index]].getVector3fMap ();

  const Eigen::Vector3f x_axis (frames_->points[index].x_axis[0],
                                frames_->points[index].x_axis[1],
                                frames_->points[index].x_axis[2]);
  //const Eigen::Vector3f& y_axis = frames_->points[index].y_axis.getNormalVector3fMap ();
  const Eigen::Vector3f normal (frames_->points[index].z_axis[0],
                                frames_->points[index].z_axis[1],
                                frames_->points[index].z_axis[2]);

  // Find every point within specified search_radius_
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  const size_t neighb_cnt = searchForNeighbors ((*indices_)[index], search_radius_, nn_indices, nn_dists);
  // For each point within radius
  for (size_t ne = 0; ne < neighb_cnt; ne++)
  {
    if (pcl::utils::equal(nn_dists[ne], 0.0f))
      continue;
    // Get neighbours coordinates
    Eigen::Vector3f neighbour = surface_->points[nn_indices[ne]].getVector3fMap ();

    // ----- Compute current neighbour polar coordinates -----

    // Get distance between the neighbour and the origin
    float r = std::sqrt (nn_dists[ne]);

    // Project point into the tangent plane
    Eigen::Vector3f proj;
    pcl::geometry::project (neighbour, origin, normal, proj);
    proj -= origin;

    // Normalize to compute the dot product
    proj.normalize ();

    // Compute the angle between the projection and the x axis in the interval [0,360]
    Eigen::Vector3f cross = x_axis.cross (proj);
    float phi = rad2deg (std::atan2 (cross.norm (), x_axis.dot (proj)));
    phi = cross.dot (normal) < 0.f ? (360.0f - phi) : phi;
    /// Compute the angle between the neighbour and the z axis (normal) in the interval [0, 180]
    Eigen::Vector3f no = neighbour - origin;
    no.normalize ();
    float theta = normal.dot (no);
    theta = pcl::rad2deg (acosf (std::min (1.0f, std::max (-1.0f, theta))));

    /// Bin (j, k, l)
    size_t j = 0;
    size_t k = 0;
    size_t l = 0;

    /// Compute the Bin(j, k, l) coordinates of current neighbour
    for (size_t rad = 1; rad < radius_bins_ + 1; rad++)
    {
      if (r <= radii_interval_[rad])
      {
        j = rad - 1;
        break;
      }
    }

    for (size_t ang = 1; ang < elevation_bins_ + 1; ang++)
    {
      if (theta <= theta_divisions_[ang])
      {
        k = ang - 1;
        break;
      }
    }

    for (size_t ang = 1; ang < azimuth_bins_ + 1; ang++)
    {
      if (phi <= phi_divisions_[ang])
      {
        l = ang - 1;
        break;
      }
    }

    /// Local point density = number of points in a sphere of radius "point_density_radius_" around the current neighbour
    std::vector<int> neighbour_indices;
    std::vector<float> neighbour_didtances;
    float point_density = static_cast<float> (searchForNeighbors (*surface_, nn_indices[ne], point_density_radius_, neighbour_indices, neighbour_didtances));
    /// point_density is always bigger than 0 because FindPointsWithinRadius returns at least the point itself
    float w = (1.0f / point_density) * volume_lut_[(l*elevation_bins_*radius_bins_) +
                                                   (k*radius_bins_) +
                                                   j];

    assert (w >= 0.0);
    if (w == std::numeric_limits<float>::infinity ())
      PCL_ERROR ("Shape Context Error INF!\n");
    if (std::isnan(w))
      PCL_ERROR ("Shape Context Error IND!\n");
    /// Accumulate w into correspondent Bin(j,k,l)
    desc[(l*elevation_bins_*radius_bins_) + (k*radius_bins_) + j] += w;

    assert (desc[(l*elevation_bins_*radius_bins_) + (k*radius_bins_) + j] >= 0);
  } // end for each neighbour
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename PointRFT> void
pcl::UniqueShapeContext<PointInT, PointOutT, PointRFT>::computeFeature (PointCloudOut &output)
{
  assert (descriptor_length_ == 1960);

  output.is_dense = true;

  for (size_t point_index = 0; point_index < indices_->size (); ++point_index)
  {
    //output[point_index].descriptor.resize (descriptor_length_);

    // If the point is not finite, set the descriptor to NaN and continue
    const PointRFT& current_frame = (*frames_)[point_index];
    if (!isFinite ((*input_)[(*indices_)[point_index]]) ||
        !std::isfinite (current_frame.x_axis[0]) ||
        !std::isfinite (current_frame.y_axis[0]) ||
        !std::isfinite (current_frame.z_axis[0])  )
    {
      for (size_t i = 0; i < descriptor_length_; ++i)
        output[point_index].descriptor[i] = std::numeric_limits<float>::quiet_NaN ();

      memset (output[point_index].rf, 0, sizeof (output[point_index].rf[0]) * 9);
      output.is_dense = false;
      continue;
    }

    for (int d = 0; d < 3; ++d)
    {
      output.points[point_index].rf[0 + d] = current_frame.x_axis[d];
      output.points[point_index].rf[3 + d] = current_frame.y_axis[d];
      output.points[point_index].rf[6 + d] = current_frame.z_axis[d];
    }

    std::vector<float> descriptor (descriptor_length_);
    computePointDescriptor (point_index, descriptor);
    for (size_t j = 0; j < descriptor_length_; ++j)
      output [point_index].descriptor[j] = descriptor[j];
  }
}

#define PCL_INSTANTIATE_UniqueShapeContext(T,OutT,RFT) template class PCL_EXPORTS pcl::UniqueShapeContext<T,OutT,RFT>;

#endif
