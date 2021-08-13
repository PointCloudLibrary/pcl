/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#ifndef PCL_FEATURES_IMPL_GRSD_H_
#define PCL_FEATURES_IMPL_GRSD_H_

#include <pcl/features/grsd.h>
#include <pcl/features/rsd.h> // for RSDEstimation
///////// STATIC /////////
template <typename PointInT, typename PointNT, typename PointOutT> int
pcl::GRSDEstimation<PointInT, PointNT, PointOutT>::getSimpleType (float min_radius, float max_radius,
                                                                  double min_radius_plane,
                                                                  double max_radius_noise,
                                                                  double min_radius_cylinder,
                                                                  double max_min_radius_diff)
{
  if (min_radius > min_radius_plane)
    return (1); // plane
  if (max_radius > min_radius_cylinder)
    return (2); // cylinder (rim)
  if (min_radius < max_radius_noise)
    return (0); // noise/corner
  if (max_radius - min_radius < max_min_radius_diff)
    return (3); // sphere/corner
  return (4);   // edge
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GRSDEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if search_radius_ was set
  if (width_ < 0)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] A voxel cell width needs to be set!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  // Create the voxel grid
  PointCloudInPtr cloud_downsampled (new PointCloudIn());
  pcl::VoxelGrid<PointInT> grid;
  grid.setLeafSize (width_, width_, width_);
  grid.setInputCloud (input_);
  grid.setSaveLeafLayout (true); // TODO maybe avoid this using nearest neighbor search
  grid.filter (*cloud_downsampled);

  // Compute RSD
  pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr radii (new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
  pcl::RSDEstimation<PointInT, PointNT, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud (cloud_downsampled);
  rsd.setSearchSurface (input_);
  rsd.setInputNormals (normals_);
  rsd.setRadiusSearch (std::max (search_radius_, std::sqrt (3.0) * width_ / 2));
  rsd.compute (*radii);

  // Save the type of each point
  int NR_CLASS = 5; // TODO make this nicer
  std::vector<int> types (radii->size ());
  std::transform(radii->points.cbegin (), radii->points.cend (), types.begin (),
    [](const auto& point) {
      // GCC 5.4 can't find unqualified getSimpleType
      return GRSDEstimation<PointInT, PointNT, PointOutT>::getSimpleType(point.r_min, point.r_max); });

  // Get the transitions between surface types between neighbors of occupied cells
  Eigen::MatrixXi transition_matrix = Eigen::MatrixXi::Zero (NR_CLASS + 1, NR_CLASS + 1);
  for (std::size_t idx = 0; idx < cloud_downsampled->size (); ++idx)
  {
    const int source_type = types[idx];
    std::vector<int> neighbors = grid.getNeighborCentroidIndices ((*cloud_downsampled)[idx], relative_coordinates_all_);
    for (const int &neighbor : neighbors)
    {
      int neighbor_type = NR_CLASS;
      if (neighbor != -1) // not empty
        neighbor_type = types[neighbor];
      transition_matrix (source_type, neighbor_type)++;
    }
  }

  // Save feature values
  output.resize (1);
  output.height = output.width = 1;
  int nrf = 0;
  for (int i = 0; i < NR_CLASS + 1; i++)
    for (int j = i; j < NR_CLASS + 1; j++)
      output[0].histogram[nrf++] = transition_matrix (i, j) + transition_matrix (j, i);
}

#define PCL_INSTANTIATE_GRSDEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::GRSDEstimation<T,NT,OutT>;

#endif /* PCL_FEATURES_IMPL_GRSD_H_ */
