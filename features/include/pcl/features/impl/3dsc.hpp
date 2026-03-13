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
#include <numeric> // partial_sum
#include <limits>  // numeric_limits

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT>
bool pcl::ShapeContext3DEstimation<PointInT, PointNT, PointOutT>::initCompute ()
{
  if (!FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  if (search_radius_ < min_radius_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] search_radius_ must be GREATER than min_radius_.\n", getClassName ().c_str ());
    return (false);
  }

  // descriptor length: elevation * azimuth * radius
  descriptor_length_ = elevation_bins_ * azimuth_bins_ * radius_bins_;

  // angular divisions in degrees
  float azimuth_interval = 360.0f / static_cast<float> (azimuth_bins_);
  float elevation_interval = 180.0f / static_cast<float> (elevation_bins_);

  // clear & reserve lookup tables
  radii_interval_.clear ();
  phi_divisions_.clear ();
  theta_divisions_.clear ();
  volume_lut_.clear ();

  // radii intervals: exponential binning per Frome paper
  radii_interval_.resize (radius_bins_ + 1);
  for (std::size_t j = 0; j < radius_bins_ + 1; j++)
    radii_interval_[j] = static_cast<float> (std::exp (
        std::log (min_radius_) +
        ((static_cast<float> (j) / static_cast<float> (radius_bins_)) * std::log (search_radius_ / min_radius_))));

  // elevation (theta) divisions cumulative
  theta_divisions_.resize (elevation_bins_ + 1, elevation_interval);
  theta_divisions_[0] = 0.f;
  std::partial_sum(theta_divisions_.begin (), theta_divisions_.end (), theta_divisions_.begin ());

  // azimuth (phi) divisions cumulative
  phi_divisions_.resize (azimuth_bins_ + 1, azimuth_interval);
  phi_divisions_[0] = 0.f;
  std::partial_sum(phi_divisions_.begin (), phi_divisions_.end (), phi_divisions_.begin ());

  // precompute bin volumes (used to normalize by bin volume and local density)
  float integr_phi  = pcl::deg2rad (phi_divisions_[1]) - pcl::deg2rad (phi_divisions_[0]);
  float e = 1.0f / 3.0f; // cube root exponent
  volume_lut_.resize (radius_bins_ * elevation_bins_ * azimuth_bins_);
  for (std::size_t j = 0; j < radius_bins_; j++)
  {
    float integr_r = (radii_interval_[j+1] * radii_interval_[j+1] * radii_interval_[j+1] / 3.0f)
                   - (radii_interval_[j]   * radii_interval_[j]   * radii_interval_[j]   / 3.0f);

    for (std::size_t k = 0; k < elevation_bins_; k++)
    {
      float integr_theta = std::cos (pcl::deg2rad (theta_divisions_[k])) - std::cos (pcl::deg2rad (theta_divisions_[k+1]));
      float V = integr_phi * integr_theta * integr_r;
      for (std::size_t l = 0; l < azimuth_bins_; l++)
      {
        // store 1 / cbrt(V) to normalize later
        volume_lut_[(l*elevation_bins_*radius_bins_) + k*radius_bins_ + j] = 1.0f / powf (V, e);
      }
    }
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT>
bool pcl::ShapeContext3DEstimation<PointInT, PointNT, PointOutT>::computePoint (
    std::size_t index, const pcl::PointCloud<PointNT> &normals, float rf[9], std::vector<float> &desc)
{
  // rf layout: [x_axis(3) | y_axis(3) | normal(3)]
  Eigen::Map<Eigen::Vector3f> x_axis (rf);
  Eigen::Map<Eigen::Vector3f> y_axis (rf + 3);
  Eigen::Map<Eigen::Vector3f> normal (rf + 6);

  // find neighbors within search_radius_
  pcl::Indices nn_indices;
  std::vector<float> nn_dists;
  const std::size_t neighb_cnt = searchForNeighbors ((*indices_)[index], search_radius_, nn_indices, nn_dists);
  if (neighb_cnt == 0)
  {
    std::fill (desc.begin (), desc.end (), std::numeric_limits<float>::quiet_NaN ());
    std::fill_n (rf, 9, 0.f);
    return (false);
  }

  const auto minDistanceIt = std::min_element(nn_dists.begin (), nn_dists.end ());
  const auto minIndex = nn_indices[std::distance (nn_dists.begin (), minDistanceIt)];

  // origin & normal (use nearest neighbor's normal)
  Vector3fMapConst origin = (*input_)[(*indices_)[index]].getVector3fMap ();
  if (!pcl::isNormalFinite(normals[minIndex]))
  {
    std::fill (desc.begin (), desc.end (), std::numeric_limits<float>::quiet_NaN ());
    std::fill (rf, rf + 9, 0.f);
    return (false);
  }
  normal = normals[minIndex].getNormalVector3fMap ();

  // build a random X axis orthogonal to normal (this is how 3DSC defines RF)
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
  assert (pcl::utils::equal (x_axis.dot(normal), 0.0f, 1E-6f));
  y_axis.matrix () = normal.cross (x_axis);

  // accumulate contribution of each neighbor into desc histogram
  for (std::size_t ne = 0; ne < neighb_cnt; ne++)
  {
    if (pcl::utils::equal (nn_dists[ne], 0.0f))
      continue;

    Eigen::Vector3f neighbour = (*surface_)[nn_indices[ne]].getVector3fMap ();

    // radial distance
    float r = std::sqrt (nn_dists[ne]);

    // projection into tangent plane and normalized
    Eigen::Vector3f proj;
    pcl::geometry::project (neighbour, origin, normal, proj);
    proj -= origin;
    proj.normalize ();

    // azimuth angle phi in [0,360]
    Eigen::Vector3f cross = x_axis.cross (proj);
    float phi = pcl::rad2deg (std::atan2 (cross.norm (), x_axis.dot (proj)));
    phi = cross.dot (normal) < 0.f ? (360.0f - phi) : phi;

    // elevation theta in [0,180]
    Eigen::Vector3f no = neighbour - origin;
    no.normalize ();
    float theta = normal.dot (no);
    theta = pcl::rad2deg (std::acos (std::min (1.0f, std::max (-1.0f, theta))));

    // find bin indices (j=radius, k=elevation, l=azimuth)
    const auto rad_min = std::lower_bound(std::next (radii_interval_.cbegin ()), radii_interval_.cend (), r);
    const auto theta_min = std::lower_bound(std::next (theta_divisions_.cbegin ()), theta_divisions_.cend (), theta);
    const auto phi_min = std::lower_bound(std::next (phi_divisions_.cbegin ()), phi_divisions_.cend (), phi);
    const auto j = std::distance(radii_interval_.cbegin (), std::prev(rad_min));
    const auto k = std::distance(theta_divisions_.cbegin (), std::prev(theta_min));
    const auto l = std::distance(phi_divisions_.cbegin (), std::prev(phi_min));

    // local point density (used to weight contributions)
    pcl::Indices neighbour_indices;
    std::vector<float> neighbour_distances;
    int point_density = searchForNeighbors (*surface_, nn_indices[ne], point_density_radius_, neighbour_indices, neighbour_distances);
    if (point_density == 0)
      continue;

    float w = (1.0f / static_cast<float> (point_density)) *
              volume_lut_[(l*elevation_bins_*radius_bins_) + (k*radius_bins_) + j];

    if (w == std::numeric_limits<float>::infinity ())
      PCL_ERROR ("Shape Context Error INF!\n");
    if (std::isnan(w))
      PCL_ERROR ("Shape Context Error IND!\n");

    // accumulate into the flat descriptor: index layout is (azimuth-block * elevation_bins * radius_bins) + (elevation * radius_bins) + radius
    desc[(l*elevation_bins_*radius_bins_) + (k*radius_bins_) + j] += w;
  } // end for each neighbour

  // 3DSC does not define a repeatable local RF; set rf to zero as a signal
  std::fill_n (rf, 9, 0);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT>
void pcl::ShapeContext3DEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  assert (descriptor_length_ == 1980);

  output.is_dense = true;
  // iterate over all requested indices
  for (std::size_t point_index = 0; point_index < indices_->size (); point_index++)
  {
    // if input point is invalid, fill descriptor with NaNs
    if (!isFinite ((*input_)[(*indices_)[point_index]]))
    {
      std::fill_n (output[point_index].descriptor, descriptor_length_, std::numeric_limits<float>::quiet_NaN ());
      std::fill_n (output[point_index].rf, 9, 0);
      output.is_dense = false;
      continue;
    }

    // compute raw descriptor for this point
    std::vector<float> descriptor (descriptor_length_);
    if (!computePoint (point_index, *normals_, output[point_index].rf, descriptor))
      output.is_dense = false;

    // ---------------------------
    // IMPORTANT: normalize azimuth orientation deterministically so that
    // descriptors do not depend on the random local X axis selection.
    // This shifts azimuth blocks so the dominant azimuth block becomes first.
    // ---------------------------
    shiftAlongAzimuth(descriptor);

    // copy into output fixed-size array
    std::copy (descriptor.cbegin (), descriptor.cend (), output[point_index].descriptor);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT>
void pcl::ShapeContext3DEstimation<PointInT, PointNT, PointOutT>::shiftAlongAzimuth (
    std::vector<float>& desc) const
{
  // Number of scalar bins inside one azimuth block (elevation_bins * radius_bins)
  const std::size_t bins_per_azimuth = elevation_bins_ * radius_bins_;

  // Find which azimuth block (0..azimuth_bins_-1) has the largest accumulated energy
  std::size_t best_azimuth = 0;
  float max_energy = -std::numeric_limits<float>::max();

  for (std::size_t a = 0; a < azimuth_bins_; ++a)
  {
    float energy = 0.0f;
    const std::size_t offset = a * bins_per_azimuth;
    for (std::size_t i = 0; i < bins_per_azimuth; ++i)
      energy += desc[offset + i];

    if (energy > max_energy)
    {
      max_energy = energy;
      best_azimuth = a;
    }
  }

  // If the dominant block is already at index 0, nothing to do
  if (best_azimuth == 0)
    return;

  // Circularly rotate the entire descriptor by shift = best_azimuth * bins_per_azimuth
  std::vector<float> rotated(desc.size());
  const std::size_t shift = best_azimuth * bins_per_azimuth;

  // rotated[i] = desc[(i + shift) % desc.size()]
  for (std::size_t i = 0; i < desc.size(); ++i)
    rotated[i] = desc[(i + shift) % desc.size()];

  desc.swap(rotated);
  // After swap, desc holds the rotated, orientation-normalized histogram.
}

#define PCL_INSTANTIATE_ShapeContext3DEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::ShapeContext3DEstimation<T,NT,OutT>;