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
 */

#pragma once

#include <pcl/features/3dsc.h>

#include <pcl/common/angles.h>
#include <pcl/common/geometry.h>
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/common/utils.h>

#include <cmath>
#include <numeric> // for partial_sum

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::ShapeContext3DEstimation<PointInT, PointNT, PointOutT>::initCompute ()
{
  if (!FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ())
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
  for (std::size_t j = 0; j < radius_bins_ + 1; j++)
    radii_interval_[j] = static_cast<float> (std::exp (std::log (min_radius_) + ((static_cast<float> (j) / static_cast<float> (radius_bins_)) * std::log (search_radius_ / min_radius_))));

  // Fill theta divisions of elevation
  theta_divisions_.resize (elevation_bins_ + 1, elevation_interval);
  theta_divisions_[0] = 0.f;
  std::partial_sum(theta_divisions_.begin (), theta_divisions_.end (), theta_divisions_.begin ());

  // Fill phi didvisions of elevation
  phi_divisions_.resize (azimuth_bins_ + 1, azimuth_interval);
  phi_divisions_[0] = 0.f;
  std::partial_sum(phi_divisions_.begin (), phi_divisions_.end (), phi_divisions_.begin ());

  // LookUp Table that contains the volume of all the bins
  // "phi" term of the volume integral
  // "integr_phi" has always the same value so we compute it only one time
  float integr_phi  = pcl::deg2rad (phi_divisions_[1]) - pcl::deg2rad (phi_divisions_[0]);
  // exponential to compute the cube root using pow
  float e = 1.0f / 3.0f;
  // Resize volume look up table
  volume_lut_.resize (radius_bins_ * elevation_bins_ * azimuth_bins_);
  // Fill volumes look up table
  for (std::size_t j = 0; j < radius_bins_; j++)
  {
    // "r" term of the volume integral
    float integr_r = (radii_interval_[j+1] * radii_interval_[j+1] * radii_interval_[j+1] / 3.0f) - (radii_interval_[j] * radii_interval_[j] * radii_interval_[j] / 3.0f);

    for (std::size_t k = 0; k < elevation_bins_; k++)
    {
      // "theta" term of the volume integral
      float integr_theta = std::cos (pcl::deg2rad (theta_divisions_[k])) - std::cos (pcl::deg2rad (theta_divisions_[k+1]));
      // Volume
      float V = integr_phi * integr_theta * integr_r;
      // Compute cube root of the computed volume commented for performance but left
      // here for clarity
      // float cbrt = pow(V, e);
      // cbrt = 1 / cbrt;

      for (std::size_t l = 0; l < azimuth_bins_; l++)
      {
        // Store in lut 1/cbrt
        //volume_lut_[ (l*elevation_bins_*radius_bins_) + k*radius_bins_ + j ] = cbrt;
        volume_lut_[(l*elevation_bins_*radius_bins_) + k*radius_bins_ + j] = 1.0f / powf (V, e);
      }
    }
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::ShapeContext3DEstimation<PointInT, PointNT, PointOutT>::computePoint (
    std::size_t index, const pcl::PointCloud<PointNT> &normals, float rf[9], std::vector<float> &desc)
{
  // The RF is formed as this x_axis | y_axis | normal
  Eigen::Map<Eigen::Vector3f> x_axis (rf);
  Eigen::Map<Eigen::Vector3f> y_axis (rf + 3);
  Eigen::Map<Eigen::Vector3f> normal (rf + 6);

  // Find every point within specified search_radius_
  pcl::Indices nn_indices;
  std::vector<float> nn_dists;
  const std::size_t neighb_cnt = searchForNeighbors ((*indices_)[index], search_radius_, nn_indices, nn_dists);
  if (neighb_cnt == 0)
  {
    std::fill (desc.begin (), desc.end (), std::numeric_limits<float>::quiet_NaN ());
    std::fill (rf, rf + 9, 0.f);
    return (false);
  }

  const auto minDistanceIt = std::min_element(nn_dists.begin (), nn_dists.end ());
  const auto minIndex = nn_indices[std::distance (nn_dists.begin (), minDistanceIt)];

  // Get origin point
  Vector3fMapConst origin = (*input_)[(*indices_)[index]].getVector3fMap ();
  // Get origin normal
  // Use pre-computed normals
  if (!pcl::isNormalFinite(normals[minIndex]))
  {
    std::fill (desc.begin (), desc.end (), std::numeric_limits<float>::quiet_NaN ());
    std::fill (rf, rf + 9, 0.f);
    return (false);
  }
  normal = normals[minIndex].getNormalVector3fMap ();

  // Compute and store the RF direction
  x_axis[0] = rnd ();
  x_axis[1] = rnd ();
  x_axis[2] = rnd ();
  if (!pcl::utils::equal (normal[2], 0.0f))
    x_axis[2] = - (normal[0]*x_axis[0] + normal[1]*x_axis[1]) / normal[2];
  else if (!pcl::utils::equal (normal[1], 0.0f))
    x_axis[1] = - (normal[0]*x_axis[0] + normal[2]*x_axis[2]) / normal[1];
  else if (!pcl::utils::equal (normal[0], 0.0f))
    x_axis[0] = - (normal[1]*x_axis[1] + normal[2]*x_axis[2]) / normal[0];

  x_axis.normalize ();

  // Check if the computed x axis is orthogonal to the normal
  assert (pcl::utils::equal (x_axis[0]*normal[0] + x_axis[1]*normal[1] + x_axis[2]*normal[2], 0.0f, 1E-6f));

  // Store the 3rd frame vector
  y_axis.matrix () = normal.cross (x_axis);

  // For each point within radius
  for (std::size_t ne = 0; ne < neighb_cnt; ne++)
  {
    if (pcl::utils::equal (nn_dists[ne], 0.0f))
		  continue;
    // Get neighbours coordinates
    Eigen::Vector3f neighbour = (*surface_)[nn_indices[ne]].getVector3fMap ();

    /// ----- Compute current neighbour polar coordinates -----
    /// Get distance between the neighbour and the origin
    float r = std::sqrt (nn_dists[ne]);

    /// Project point into the tangent plane
    Eigen::Vector3f proj;
    pcl::geometry::project (neighbour, origin, normal, proj);
    proj -= origin;

    /// Normalize to compute the dot product
    proj.normalize ();

    /// Compute the angle between the projection and the x axis in the interval [0,360]
    Eigen::Vector3f cross = x_axis.cross (proj);
    float phi = pcl::rad2deg (std::atan2 (cross.norm (), x_axis.dot (proj)));
    phi = cross.dot (normal) < 0.f ? (360.0f - phi) : phi;
    /// Compute the angle between the neighbour and the z axis (normal) in the interval [0, 180]
    Eigen::Vector3f no = neighbour - origin;
    no.normalize ();
    float theta = normal.dot (no);
    theta = pcl::rad2deg (std::acos (std::min (1.0f, std::max (-1.0f, theta))));

    // Compute the Bin(j, k, l) coordinates of current neighbour
    const auto rad_min = std::lower_bound(std::next (radii_interval_.cbegin ()), radii_interval_.cend (), r);
    const auto theta_min = std::lower_bound(std::next (theta_divisions_.cbegin ()), theta_divisions_.cend (), theta);
    const auto phi_min = std::lower_bound(std::next (phi_divisions_.cbegin ()), phi_divisions_.cend (), phi);

    // Bin (j, k, l)
    const auto j = std::distance(radii_interval_.cbegin (), std::prev(rad_min));
    const auto k = std::distance(theta_divisions_.cbegin (), std::prev(theta_min));
    const auto l = std::distance(phi_divisions_.cbegin (), std::prev(phi_min));

    // Local point density = number of points in a sphere of radius "point_density_radius_" around the current neighbour
    pcl::Indices neighbour_indices;
    std::vector<float> neighbour_distances;
    int point_density = searchForNeighbors (*surface_, nn_indices[ne], point_density_radius_, neighbour_indices, neighbour_distances);
    // point_density is NOT always bigger than 0 (on error, searchForNeighbors returns 0), so we must check for that
    if (point_density == 0)
      continue;

    float w = (1.0f / static_cast<float> (point_density)) *
              volume_lut_[(l*elevation_bins_*radius_bins_) +  (k*radius_bins_) + j];

    assert (w >= 0.0);
    if (w == std::numeric_limits<float>::infinity ())
      PCL_ERROR ("Shape Context Error INF!\n");
    if (std::isnan(w))
      PCL_ERROR ("Shape Context Error IND!\n");
    /// Accumulate w into correspondent Bin(j,k,l)
    desc[(l*elevation_bins_*radius_bins_) + (k*radius_bins_) + j] += w;

    assert (desc[(l*elevation_bins_*radius_bins_) + (k*radius_bins_) + j] >= 0);
  } // end for each neighbour

  // 3DSC does not define a repeatable local RF, we set it to zero to signal it to the user
  std::fill (rf, rf + 9, 0);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::ShapeContext3DEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  assert (descriptor_length_ == 1980);

  output.is_dense = true;
  // Iterate over all points and compute the descriptors
	for (std::size_t point_index = 0; point_index < indices_->size (); point_index++)
  {
    //output[point_index].descriptor.resize (descriptor_length_);

    // If the point is not finite, set the descriptor to NaN and continue
    if (!isFinite ((*input_)[(*indices_)[point_index]]))
    {
      std::fill (output[point_index].descriptor, output[point_index].descriptor + descriptor_length_,
                 std::numeric_limits<float>::quiet_NaN ());
      std::fill (output[point_index].rf, output[point_index].rf + 9, 0);
      output.is_dense = false;
      continue;
    }

    std::vector<float> descriptor (descriptor_length_);
    if (!computePoint (point_index, *normals_, output[point_index].rf, descriptor))
      output.is_dense = false;
    std::copy (descriptor.begin (), descriptor.end (), output[point_index].descriptor);
  }
}

#define PCL_INSTANTIATE_ShapeContext3DEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::ShapeContext3DEstimation<T,NT,OutT>;

