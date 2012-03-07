/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_SURFACE_IMPL_MLS_OMP_H_
#define PCL_SURFACE_IMPL_MLS_OMP_H_

#include <cstddef>
#include <pcl/surface/mls_omp.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename NormalOutT> void
pcl::MovingLeastSquaresOMP<PointInT, NormalOutT>::performReconstruction (PointCloudIn &output)
{
  typedef std::size_t size_t;
  // Compute the number of coefficients
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

  // Allocate enough space to hold the results of nearest neighbor searches
  // \note resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices;
  std::vector<float> nn_sqr_dists;

//#pragma omp parallel for schedule (dynamic, threads_)
  // For all points
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    // Get the initial estimates of point positions and their neighborhoods
//    if (!searchForNeighbors ((*indices_)[cp], nn_indices, nn_sqr_dists))
//      continue;

    //printf ("search_radius_ %f", search_radius_);
    //printf ("index to search: %d --- %f %f %f\n", (*indices_)[cp], input_->points[(*indices_)[cp]].x, input_->points[(*indices_)[cp]].y, input_->points[(*indices_)[cp]].z);
    if (!tree_->radiusSearch ((*indices_)[cp], search_radius_, nn_indices, nn_sqr_dists))
      continue;

    // Check the number of nearest neighbors for normal estimation (and later
    // for polynomial fit as well)
    if (nn_indices.size () < 3)
      continue;


    PointCloudIn projected_points;
    NormalCloudOut projected_points_normals;

    // Get a plane approximating the local surface's tangent and project point onto it
    computeMLSPointNormal ((*indices_)[cp], *input_, nn_indices, nn_sqr_dists, projected_points, projected_points_normals);

    // Append projected points to output
    output.insert (output.end (), projected_points.begin (), projected_points.end ());
    normals_->insert (normals_->end (), projected_points_normals.begin (), projected_points_normals.end ());
  }


  // For the voxel grid upsampling method, generate the voxel grid and dilate it
  // Then, project the newly obtained points to the MLS surface
  if (upsample_method_ == MovingLeastSquares<PointInT, NormalOutT>::VOXEL_GRID_DILATION)
  {
    MLSVoxelGrid voxel_grid (input_, indices_, voxel_size_);

    for (int iteration = 0; iteration < dilation_iteration_num_; ++iteration)
      voxel_grid.dilate ();

// TODO: there seems no way of putting an OpenMP directive in front of BOOST_FOREACH ?
    BOOST_FOREACH (typename MLSVoxelGrid::HashMap::value_type voxel, voxel_grid.voxel_grid_)
    {
      // Get 3D position of point
      Eigen::Vector3f pos;
      voxel_grid.getPosition (voxel.first, pos);

      PointInT p;
      p.x = pos[0];
      p.y = pos[1];
      p.z = pos[2];

      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->nearestKSearch (p, 1, nn_indices, nn_dists);
      int input_index = nn_indices.front ();

      // If the closest point did not have a valid MLS fitting result
      // OR if it is too far away from the sampled point
      if (mls_results_[input_index].valid == false)
        continue;

      Eigen::Vector3f add_point = p.getVector3fMap (),
                      input_point = input_->points[input_index].getVector3fMap ();

      Eigen::Vector3d aux = mls_results_[input_index].u;
      Eigen::Vector3f u = aux.cast<float> ();
      aux = mls_results_[input_index].v;
      Eigen::Vector3f v = aux.cast<float> ();

      float u_disp = (add_point - input_point).dot (u),
            v_disp = (add_point - input_point).dot (v);

      PointInT result_point;
      NormalOutT result_normal;
      projectPointToMLSSurface (u_disp, v_disp,
                                mls_results_[input_index].u, mls_results_[input_index].v,
                                mls_results_[input_index].plane_normal,
                                mls_results_[input_index].curvature,
                                input_point,
                                mls_results_[input_index].c_vec,
                                mls_results_[input_index].num_neighbors,
                                result_point, result_normal);

      float d_before = (pos - input_point).norm (),
            d_after = (result_point.getVector3fMap () - input_point). norm();
      if (d_after > d_before)
        continue;

      output.push_back (result_point);
      normals_->push_back (result_normal);
    }
  }


  // Set proper widths and heights for the clouds
  normals_->height = 1;
  normals_->width = static_cast<uint32_t> (normals_->size ());
  output.height = 1;
  output.width = static_cast<uint32_t> (output.size ());
}

#define PCL_INSTANTIATE_MovingLeastSquaresOMP(T,OutT) template class PCL_EXPORTS pcl::MovingLeastSquaresOMP<T,OutT>;

#endif    // PCL_SURFACE_IMPL_MLS_OMP_H_

