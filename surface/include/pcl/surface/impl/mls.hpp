/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_SURFACE_IMPL_MLS_H_
#define PCL_SURFACE_IMPL_MLS_H_

#include <pcl/type_traits.h>
#include <pcl/surface/mls.h>
#include <pcl/common/common.h> // for getMinMax3D
#include <pcl/common/copy_point.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/search/organized.h> // for OrganizedNeighbor

#include <Eigen/Geometry> // for cross
#include <Eigen/LU> // for inverse

#ifdef _OPENMP
#include <omp.h>
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::process (PointCloudOut &output)
{
  // Reset or initialize the collection of indices
  corresponding_input_indices_.reset (new PointIndices);

  // Check if normals have to be computed/saved
  if (compute_normals_)
  {
    normals_.reset (new NormalCloud);
    // Copy the header
    normals_->header = input_->header;
    // Clear the fields in case the method exits before computation
    normals_->width = normals_->height = 0;
    normals_->points.clear ();
  }

  // Copy the header
  output.header = input_->header;
  output.width = output.height = 0;
  output.clear ();

  if (search_radius_ <= 0 || sqr_gauss_param_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::process] Invalid search radius (%f) or Gaussian parameter (%f)!\n", getClassName ().c_str (), search_radius_, sqr_gauss_param_);
    return;
  }

  // Check if distinct_cloud_ was set
  if (upsample_method_ == DISTINCT_CLOUD && !distinct_cloud_)
  {
    PCL_ERROR ("[pcl::%s::process] Upsample method was set to DISTINCT_CLOUD, but no distinct cloud was specified.\n", getClassName ().c_str ());
    return;
  }

  if (!initCompute ())
    return;

  // Initialize the spatial locator
  if (!tree_)
  {
    KdTreePtr tree;
    if (input_->isOrganized ())
      tree.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
    else
      tree.reset (new pcl::search::KdTree<PointInT> (false));
    setSearchMethod (tree);
  }

  // Send the surface dataset to the spatial locator
  tree_->setInputCloud (input_);

  switch (upsample_method_)
  {
    // Initialize random number generator if necessary
    case (RANDOM_UNIFORM_DENSITY):
    {
      std::random_device rd;
      rng_.seed (rd());
      const double tmp = search_radius_ / 2.0;
      rng_uniform_distribution_.reset (new std::uniform_real_distribution<> (-tmp, tmp));

      break;
    }
    case (VOXEL_GRID_DILATION):
    case (DISTINCT_CLOUD):
    {
      if (!cache_mls_results_)
        PCL_WARN ("The cache mls results is forced when using upsampling method VOXEL_GRID_DILATION or DISTINCT_CLOUD.\n");

      cache_mls_results_ = true;
      break;
    }
    default:
      break;
  }

  if (cache_mls_results_)
  {
    mls_results_.resize (input_->size ());
  }
  else
  {
    mls_results_.resize (1); // Need to have a reference to a single dummy result.
  }

  // Perform the actual surface reconstruction
  performProcessing (output);

  if (compute_normals_)
  {
    normals_->height = 1;
    normals_->width = normals_->size ();

    for (std::size_t i = 0; i < output.size (); ++i)
    {
      using FieldList = typename pcl::traits::fieldList<PointOutT>::type;
      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output[i], "normal_x", (*normals_)[i].normal_x));
      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output[i], "normal_y", (*normals_)[i].normal_y));
      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output[i], "normal_z", (*normals_)[i].normal_z));
      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output[i], "curvature", (*normals_)[i].curvature));
    }

  }

  // Set proper widths and heights for the clouds
  output.height = 1;
  output.width = output.size ();

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::computeMLSPointNormal (pcl::index_t index,
                                                                     const pcl::Indices &nn_indices,
                                                                     PointCloudOut &projected_points,
                                                                     NormalCloud &projected_points_normals,
                                                                     PointIndices &corresponding_input_indices,
                                                                     MLSResult &mls_result) const
{
  // Note: this method is const because it needs to be thread-safe
  //       (MovingLeastSquaresOMP calls it from multiple threads)

  mls_result.computeMLSSurface<PointInT> (*input_, index, nn_indices, search_radius_, order_);

  switch (upsample_method_)
  {
    case (NONE):
    {
      const MLSResult::MLSProjectionResults proj = mls_result.projectQueryPoint (projection_method_, nr_coeff_);
      addProjectedPointNormal (index, proj.point, proj.normal, mls_result.curvature, projected_points, projected_points_normals, corresponding_input_indices);
      break;
    }

    case (SAMPLE_LOCAL_PLANE):
    {
      // Uniformly sample a circle around the query point using the radius and step parameters
      for (float u_disp = -static_cast<float> (upsampling_radius_); u_disp <= upsampling_radius_; u_disp += static_cast<float> (upsampling_step_))
        for (float v_disp = -static_cast<float> (upsampling_radius_); v_disp <= upsampling_radius_; v_disp += static_cast<float> (upsampling_step_))
          if (u_disp * u_disp + v_disp * v_disp < upsampling_radius_ * upsampling_radius_)
          {
            MLSResult::MLSProjectionResults proj = mls_result.projectPointSimpleToPolynomialSurface (u_disp, v_disp);
            addProjectedPointNormal (index, proj.point, proj.normal, mls_result.curvature, projected_points, projected_points_normals, corresponding_input_indices);
          }
      break;
    }

    case (RANDOM_UNIFORM_DENSITY):
    {
      // Compute the local point density and add more samples if necessary
      const int num_points_to_add = static_cast<int> (std::floor (desired_num_points_in_radius_ / 2.0 / static_cast<double> (nn_indices.size ())));

      // Just add the query point, because the density is good
      if (num_points_to_add <= 0)
      {
        // Just add the current point
        const MLSResult::MLSProjectionResults proj = mls_result.projectQueryPoint (projection_method_, nr_coeff_);
        addProjectedPointNormal (index, proj.point, proj.normal, mls_result.curvature, projected_points, projected_points_normals, corresponding_input_indices);
      }
      else
      {
        // Sample the local plane
        for (int num_added = 0; num_added < num_points_to_add;)
        {
          const double u = (*rng_uniform_distribution_) (rng_);
          const double v = (*rng_uniform_distribution_) (rng_);

          // Check if inside circle; if not, try another coin flip
          if (u * u + v * v > search_radius_ * search_radius_ / 4)
            continue;

          MLSResult::MLSProjectionResults proj;
          if (order_ > 1 && mls_result.num_neighbors >= 5 * nr_coeff_)
            proj = mls_result.projectPointSimpleToPolynomialSurface (u, v);
          else
            proj = mls_result.projectPointToMLSPlane (u, v);

          addProjectedPointNormal (index, proj.point, proj.normal, mls_result.curvature, projected_points, projected_points_normals, corresponding_input_indices);

          num_added++;
        }
      }
      break;
    }

    default:
      break;
  }
}

template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::addProjectedPointNormal (pcl::index_t index,
                                                                       const Eigen::Vector3d &point,
                                                                       const Eigen::Vector3d &normal,
                                                                       double curvature,
                                                                       PointCloudOut &projected_points,
                                                                       NormalCloud &projected_points_normals,
                                                                       PointIndices &corresponding_input_indices) const
{
  PointOutT aux;
  aux.x = static_cast<float> (point[0]);
  aux.y = static_cast<float> (point[1]);
  aux.z = static_cast<float> (point[2]);

  // Copy additional point information if available
  copyMissingFields ((*input_)[index], aux);

  projected_points.push_back (aux);
  corresponding_input_indices.indices.push_back (index);

  if (compute_normals_)
  {
    pcl::Normal aux_normal;
    aux_normal.normal_x = static_cast<float> (normal[0]);
    aux_normal.normal_y = static_cast<float> (normal[1]);
    aux_normal.normal_z = static_cast<float> (normal[2]);
    aux_normal.curvature = curvature;
    projected_points_normals.push_back (aux_normal);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::performProcessing (PointCloudOut &output)
{
  // Compute the number of coefficients
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

#ifdef _OPENMP
  // (Maximum) number of threads
  const unsigned int threads = threads_ == 0 ? 1 : threads_;
  // Create temporaries for each thread in order to avoid synchronization
  typename PointCloudOut::CloudVectorType projected_points (threads);
  typename NormalCloud::CloudVectorType projected_points_normals (threads);
  std::vector<PointIndices> corresponding_input_indices (threads);
#endif

  // For all points
#pragma omp parallel for \
  default(none) \
  shared(corresponding_input_indices, projected_points, projected_points_normals) \
  schedule(dynamic,1000) \
  num_threads(threads)
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    // Allocate enough space to hold the results of nearest neighbor searches
    // \note resize is irrelevant for a radiusSearch ().
    pcl::Indices nn_indices;
    std::vector<float> nn_sqr_dists;

    // Get the initial estimates of point positions and their neighborhoods
    if (searchForNeighbors ((*indices_)[cp], nn_indices, nn_sqr_dists))
    {
      // Check the number of nearest neighbors for normal estimation (and later for polynomial fit as well)
      if (nn_indices.size () >= 3)
      {
        // This thread's ID (range 0 to threads-1)
#ifdef _OPENMP
        const int tn = omp_get_thread_num ();
        // Size of projected points before computeMLSPointNormal () adds points
        std::size_t pp_size = projected_points[tn].size ();
#else
        PointCloudOut projected_points;
        NormalCloud projected_points_normals;
#endif

        // Get a plane approximating the local surface's tangent and project point onto it
        const int index = (*indices_)[cp];

        std::size_t mls_result_index = 0;
        if (cache_mls_results_)
          mls_result_index = index; // otherwise we give it a dummy location.

#ifdef _OPENMP
        computeMLSPointNormal (index, nn_indices, projected_points[tn], projected_points_normals[tn], corresponding_input_indices[tn], mls_results_[mls_result_index]);

        // Copy all information from the input cloud to the output points (not doing any interpolation)
        for (std::size_t pp = pp_size; pp < projected_points[tn].size (); ++pp)
          copyMissingFields ((*input_)[(*indices_)[cp]], projected_points[tn][pp]);
#else
        computeMLSPointNormal (index, nn_indices, projected_points, projected_points_normals, *corresponding_input_indices_, mls_results_[mls_result_index]);

        // Append projected points to output
        output.insert (output.end (), projected_points.begin (), projected_points.end ());
        if (compute_normals_)
          normals_->insert (normals_->end (), projected_points_normals.begin (), projected_points_normals.end ());
#endif
      }
    }
  }

#ifdef _OPENMP
  // Combine all threads' results into the output vectors
  for (unsigned int tn = 0; tn < threads; ++tn)
  {
    output.insert (output.end (), projected_points[tn].begin (), projected_points[tn].end ());
    corresponding_input_indices_->indices.insert (corresponding_input_indices_->indices.end (),
                                                  corresponding_input_indices[tn].indices.begin (), corresponding_input_indices[tn].indices.end ());
    if (compute_normals_)
      normals_->insert (normals_->end (), projected_points_normals[tn].begin (), projected_points_normals[tn].end ());
  }
#endif

  // Perform the distinct-cloud or voxel-grid upsampling
  performUpsampling (output);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::performUpsampling (PointCloudOut &output)
{

  if (upsample_method_ == DISTINCT_CLOUD)
  {
    corresponding_input_indices_.reset (new PointIndices);
    for (std::size_t dp_i = 0; dp_i < distinct_cloud_->size (); ++dp_i) // dp_i = distinct_point_i
    {
      // Distinct cloud may have nan points, skip them
      if (!std::isfinite ((*distinct_cloud_)[dp_i].x))
        continue;

      // Get 3D position of point
      //Eigen::Vector3f pos = (*distinct_cloud_)[dp_i].getVector3fMap ();
      pcl::Indices nn_indices;
      std::vector<float> nn_dists;
      tree_->nearestKSearch ((*distinct_cloud_)[dp_i], 1, nn_indices, nn_dists);
      const auto input_index = nn_indices.front ();

      // If the closest point did not have a valid MLS fitting result
      // OR if it is too far away from the sampled point
      if (mls_results_[input_index].valid == false)
        continue;

      Eigen::Vector3d add_point = (*distinct_cloud_)[dp_i].getVector3fMap ().template cast<double> ();
      MLSResult::MLSProjectionResults proj =  mls_results_[input_index].projectPoint (add_point, projection_method_,  5 * nr_coeff_);
      addProjectedPointNormal (input_index, proj.point, proj.normal, mls_results_[input_index].curvature, output, *normals_, *corresponding_input_indices_);
    }
  }

  // For the voxel grid upsampling method, generate the voxel grid and dilate it
  // Then, project the newly obtained points to the MLS surface
  if (upsample_method_ == VOXEL_GRID_DILATION)
  {
    corresponding_input_indices_.reset (new PointIndices);

    MLSVoxelGrid voxel_grid (input_, indices_, voxel_size_);
    for (int iteration = 0; iteration < dilation_iteration_num_; ++iteration)
      voxel_grid.dilate ();

    for (typename MLSVoxelGrid::HashMap::iterator m_it = voxel_grid.voxel_grid_.begin (); m_it != voxel_grid.voxel_grid_.end (); ++m_it)
    {
      // Get 3D position of point
      Eigen::Vector3f pos;
      voxel_grid.getPosition (m_it->first, pos);

      PointInT p;
      p.x = pos[0];
      p.y = pos[1];
      p.z = pos[2];

      pcl::Indices nn_indices;
      std::vector<float> nn_dists;
      tree_->nearestKSearch (p, 1, nn_indices, nn_dists);
      const auto input_index = nn_indices.front ();

      // If the closest point did not have a valid MLS fitting result
      // OR if it is too far away from the sampled point
      if (mls_results_[input_index].valid == false)
        continue;

      Eigen::Vector3d add_point = p.getVector3fMap ().template cast<double> ();
      MLSResult::MLSProjectionResults proj = mls_results_[input_index].projectPoint (add_point, projection_method_,  5 * nr_coeff_);
      addProjectedPointNormal (input_index, proj.point, proj.normal, mls_results_[input_index].curvature, output, *normals_, *corresponding_input_indices_);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::MLSResult::MLSResult (const Eigen::Vector3d &a_query_point,
                           const Eigen::Vector3d &a_mean,
                           const Eigen::Vector3d &a_plane_normal,
                           const Eigen::Vector3d &a_u,
                           const Eigen::Vector3d &a_v,
                           const Eigen::VectorXd &a_c_vec,
                           const int a_num_neighbors,
                           const float a_curvature,
                           const int a_order) :
  query_point (a_query_point), mean (a_mean), plane_normal (a_plane_normal), u_axis (a_u), v_axis (a_v), c_vec (a_c_vec), num_neighbors (a_num_neighbors),
  curvature (a_curvature), order (a_order), valid (true)
{}

void
pcl::MLSResult::getMLSCoordinates (const Eigen::Vector3d &pt, double &u, double &v, double &w) const
{
  Eigen::Vector3d delta = pt - mean;
  u = delta.dot (u_axis);
  v = delta.dot (v_axis);
  w = delta.dot (plane_normal);
}

void
pcl::MLSResult::getMLSCoordinates (const Eigen::Vector3d &pt, double &u, double &v) const
{
  Eigen::Vector3d delta = pt - mean;
  u = delta.dot (u_axis);
  v = delta.dot (v_axis);
}

double
pcl::MLSResult::getPolynomialValue (const double u, const double v) const
{
  // Compute the polynomial's terms at the current point
  // Example for second order: z = a + b*y + c*y^2 + d*x + e*x*y + f*x^2
  int j = 0;
  double u_pow = 1;
  double result = 0;
  for (int ui = 0; ui <= order; ++ui)
  {
    double v_pow = 1;
    for (int vi = 0; vi <= order - ui; ++vi)
    {
      result += c_vec[j++] * u_pow * v_pow;
      v_pow *= v;
    }
    u_pow *= u;
  }

  return (result);
}

pcl::MLSResult::PolynomialPartialDerivative
pcl::MLSResult::getPolynomialPartialDerivative (const double u, const double v) const
{
  // Compute the displacement along the normal using the fitted polynomial
  // and compute the partial derivatives needed for estimating the normal
  PolynomialPartialDerivative d{};
  Eigen::VectorXd u_pow (order + 2), v_pow (order + 2);
  int j = 0;

  d.z = d.z_u = d.z_v = d.z_uu = d.z_vv = d.z_uv = 0;
  u_pow (0) = v_pow (0) = 1;
  for (int ui = 0; ui <= order; ++ui)
  {
    for (int vi = 0; vi <= order - ui; ++vi)
    {
      // Compute displacement along normal
      d.z += u_pow (ui) * v_pow (vi) * c_vec[j];

      // Compute partial derivatives
      if (ui >= 1)
        d.z_u += c_vec[j] * ui * u_pow (ui - 1) * v_pow (vi);

      if (vi >= 1)
        d.z_v += c_vec[j] * vi * u_pow (ui) * v_pow (vi - 1);

      if (ui >= 1 && vi >= 1)
        d.z_uv += c_vec[j] * ui * u_pow (ui - 1) * vi * v_pow (vi - 1);

      if (ui >= 2)
        d.z_uu += c_vec[j] * ui * (ui - 1) * u_pow (ui - 2) * v_pow (vi);

      if (vi >= 2)
        d.z_vv += c_vec[j] * vi * (vi - 1) * u_pow (ui) * v_pow (vi - 2);

      if (ui == 0)
        v_pow (vi + 1) = v_pow (vi) * v;

      ++j;
    }
    u_pow (ui + 1) = u_pow (ui) * u;
  }

  return (d);
}

pcl::MLSResult::MLSProjectionResults
pcl::MLSResult::projectPointOrthogonalToPolynomialSurface (const double u, const double v, const double w) const
{
  double gu = u;
  double gv = v;
  double gw = 0;

  MLSProjectionResults result;
  result.normal = plane_normal;
  if (order > 1 && c_vec.size () >= (order + 1) * (order + 2) / 2 && std::isfinite (c_vec[0]))
  {
    PolynomialPartialDerivative d = getPolynomialPartialDerivative (gu, gv);
    gw = d.z;
    double err_total;
    const double dist1 = std::abs (gw - w);
    double dist2;
    do
    {
      double e1 = (gu - u) + d.z_u * gw - d.z_u * w;
      double e2 = (gv - v) + d.z_v * gw - d.z_v * w;

      const double F1u = 1 + d.z_uu * gw + d.z_u * d.z_u - d.z_uu * w;
      const double F1v = d.z_uv * gw + d.z_u * d.z_v - d.z_uv * w;

      const double F2u = d.z_uv * gw + d.z_v * d.z_u - d.z_uv * w;
      const double F2v = 1 + d.z_vv * gw + d.z_v * d.z_v - d.z_vv * w;

      Eigen::MatrixXd J (2, 2);
      J (0, 0) = F1u;
      J (0, 1) = F1v;
      J (1, 0) = F2u;
      J (1, 1) = F2v;

      Eigen::Vector2d err (e1, e2);
      Eigen::Vector2d update = J.inverse () * err;
      gu -= update (0);
      gv -= update (1);

      d = getPolynomialPartialDerivative (gu, gv);
      gw = d.z;
      dist2 = std::sqrt ((gu - u) * (gu - u) + (gv - v) * (gv - v) + (gw - w) * (gw - w));

      err_total = std::sqrt (e1 * e1 + e2 * e2);

    } while (err_total > 1e-8 && dist2 < dist1);

    if (dist2 > dist1) // the optimization was diverging reset the coordinates for simple projection
    {
      gu = u;
      gv = v;
      d = getPolynomialPartialDerivative (u, v);
      gw = d.z;
    }

    result.u = gu;
    result.v = gv;
    result.normal -= (d.z_u * u_axis + d.z_v * v_axis);
    result.normal.normalize ();
  }

  result.point = mean + gu * u_axis + gv * v_axis + gw * plane_normal;

  return (result);
}

pcl::MLSResult::MLSProjectionResults
pcl::MLSResult::projectPointToMLSPlane (const double u, const double v) const
{
  MLSProjectionResults result;
  result.u = u;
  result.v = v;
  result.normal = plane_normal;
  result.point = mean + u * u_axis + v * v_axis;

  return (result);
}

pcl::MLSResult::MLSProjectionResults
pcl::MLSResult::projectPointSimpleToPolynomialSurface (const double u, const double v) const
{
  MLSProjectionResults result;
  double w = 0;

  result.u = u;
  result.v = v;
  result.normal = plane_normal;

  if (order > 1 && c_vec.size () >= (order + 1) * (order + 2) / 2 && std::isfinite (c_vec[0]))
  {
    const PolynomialPartialDerivative d = getPolynomialPartialDerivative (u, v);
    w = d.z;
    result.normal -= (d.z_u * u_axis + d.z_v * v_axis);
    result.normal.normalize ();
  }

  result.point = mean + u * u_axis + v * v_axis + w * plane_normal;

  return (result);
}

pcl::MLSResult::MLSProjectionResults
pcl::MLSResult::projectPoint (const Eigen::Vector3d &pt, ProjectionMethod method, int required_neighbors) const
{
  double u, v, w;
  getMLSCoordinates (pt, u, v, w);

  MLSResult::MLSProjectionResults proj;
  if (order > 1 && num_neighbors >= required_neighbors && std::isfinite (c_vec[0]) && method != NONE)
  {
    if (method == ORTHOGONAL)
      proj = projectPointOrthogonalToPolynomialSurface (u, v, w);
    else // SIMPLE
      proj = projectPointSimpleToPolynomialSurface (u, v);
  }
  else
  {
    proj = projectPointToMLSPlane (u, v);
  }

  return  (proj);
}

pcl::MLSResult::MLSProjectionResults
pcl::MLSResult::projectQueryPoint (ProjectionMethod method, int required_neighbors) const
{
  MLSResult::MLSProjectionResults proj;
  if (order > 1 && num_neighbors >= required_neighbors && std::isfinite (c_vec[0]) && method != NONE)
  {
    if (method == ORTHOGONAL)
    {
      double u, v, w;
      getMLSCoordinates (query_point, u, v, w);
      proj = projectPointOrthogonalToPolynomialSurface (u, v, w);
    }
    else // SIMPLE
    {
      // Projection onto MLS surface along Darboux normal to the height at (0,0)
      proj.point = mean + (c_vec[0] * plane_normal);

      // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec[order_+1] and c_vec[1]
      proj.normal = plane_normal - c_vec[order + 1] * u_axis - c_vec[1] * v_axis;
      proj.normal.normalize ();
    }
  }
  else
  {
    proj.normal = plane_normal;
    proj.point = mean;
  }

  return (proj);
}

template <typename PointT> void
pcl::MLSResult::computeMLSSurface (const pcl::PointCloud<PointT> &cloud,
                                   pcl::index_t index,
                                   const pcl::Indices &nn_indices,
                                   double search_radius,
                                   int polynomial_order,
                                   std::function<double(const double)> weight_func)
{
  // Compute the plane coefficients
  EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
  Eigen::Vector4d xyz_centroid;

  // Estimate the XYZ centroid
  pcl::compute3DCentroid (cloud, nn_indices, xyz_centroid);

  // Compute the 3x3 covariance matrix
  pcl::computeCovarianceMatrix (cloud, nn_indices, xyz_centroid, covariance_matrix);
  EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3d eigen_vector;
  Eigen::Vector4d model_coefficients (0, 0, 0, 0);
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
  model_coefficients.head<3> ().matrix () = eigen_vector;
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  query_point = cloud[index].getVector3fMap ().template cast<double> ();

  if (!std::isfinite(eigen_vector[0]) || !std::isfinite(eigen_vector[1]) || !std::isfinite(eigen_vector[2]))
  {
    // Invalid plane coefficients, this may happen if the input cloud is non-dense (it contains invalid points).
    // Keep the input point and stop here.
    valid = false;
    mean = query_point;
    return;
  }

  // Projected query point
  valid = true;
  const double distance = query_point.dot (model_coefficients.head<3> ()) + model_coefficients[3];
  mean = query_point - distance * model_coefficients.head<3> ();

  curvature = covariance_matrix.trace ();
  // Compute the curvature surface change
  if (curvature != 0)
    curvature = std::abs (eigen_value / curvature);

  // Get a copy of the plane normal easy access
  plane_normal = model_coefficients.head<3> ();

  // Local coordinate system (Darboux frame)
  v_axis = plane_normal.unitOrthogonal ();
  u_axis = plane_normal.cross (v_axis);

  // Perform polynomial fit to update point and normal
  ////////////////////////////////////////////////////
  num_neighbors = static_cast<int> (nn_indices.size ());
  order = polynomial_order;
  if (order > 1)
  {
    const int nr_coeff = (order + 1) * (order + 2) / 2;

    if (num_neighbors >= nr_coeff)
    {
      if (!weight_func)
        weight_func = [=] (const double sq_dist) { return this->computeMLSWeight (sq_dist, search_radius * search_radius); };

      // Allocate matrices and vectors to hold the data used for the polynomial fit
      Eigen::VectorXd weight_vec (num_neighbors);
      Eigen::MatrixXd P (nr_coeff, num_neighbors);
      Eigen::VectorXd f_vec (num_neighbors);
      Eigen::MatrixXd P_weight_Pt (nr_coeff, nr_coeff);

      // Update neighborhood, since point was projected, and computing relative
      // positions. Note updating only distances for the weights for speed
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > de_meaned (num_neighbors);
      for (std::size_t ni = 0; ni < static_cast<std::size_t>(num_neighbors); ++ni)
      {
        de_meaned[ni][0] = cloud[nn_indices[ni]].x - mean[0];
        de_meaned[ni][1] = cloud[nn_indices[ni]].y - mean[1];
        de_meaned[ni][2] = cloud[nn_indices[ni]].z - mean[2];
        weight_vec (ni) = weight_func (de_meaned[ni].dot (de_meaned[ni]));
      }

      // Go through neighbors, transform them in the local coordinate system,
      // save height and the evaluation of the polynomial's terms
      for (std::size_t ni = 0; ni < static_cast<std::size_t>(num_neighbors); ++ni)
      {
        // Transforming coordinates
        const double u_coord = de_meaned[ni].dot(u_axis);
        const double v_coord = de_meaned[ni].dot(v_axis);
        f_vec (ni) = de_meaned[ni].dot (plane_normal);

        // Compute the polynomial's terms at the current point
        int j = 0;
        double u_pow = 1;
        for (int ui = 0; ui <= order; ++ui)
        {
          double v_pow = 1;
          for (int vi = 0; vi <= order - ui; ++vi)
          {
            P (j++, ni) = u_pow * v_pow;
            v_pow *= v_coord;
          }
          u_pow *= u_coord;
        }
      }

      // Computing coefficients
      const Eigen::MatrixXd P_weight = P * weight_vec.asDiagonal(); // size will be (nr_coeff_, nn_indices.size ());
      P_weight_Pt = P_weight * P.transpose ();
      c_vec = P_weight * f_vec;
      P_weight_Pt.llt ().solveInPlace (c_vec);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
pcl::MovingLeastSquares<PointInT, PointOutT>::MLSVoxelGrid::MLSVoxelGrid (PointCloudInConstPtr& cloud,
                                                                          IndicesPtr &indices,
                                                                          float voxel_size) :
  voxel_grid_ (), data_size_ (), voxel_size_ (voxel_size)
{
  pcl::getMinMax3D (*cloud, *indices, bounding_min_, bounding_max_);

  Eigen::Vector4f bounding_box_size = bounding_max_ - bounding_min_;
  const double max_size = (std::max) ((std::max)(bounding_box_size.x (), bounding_box_size.y ()), bounding_box_size.z ());
  // Put initial cloud in voxel grid
  data_size_ = static_cast<std::uint64_t> (1.5 * max_size / voxel_size_);
  for (std::size_t i = 0; i < indices->size (); ++i)
    if (std::isfinite ((*cloud)[(*indices)[i]].x))
    {
      Eigen::Vector3i pos;
      getCellIndex ((*cloud)[(*indices)[i]].getVector3fMap (), pos);

      std::uint64_t index_1d;
      getIndexIn1D (pos, index_1d);
      Leaf leaf;
      voxel_grid_[index_1d] = leaf;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::MLSVoxelGrid::dilate ()
{
  HashMap new_voxel_grid = voxel_grid_;
  for (typename MLSVoxelGrid::HashMap::iterator m_it = voxel_grid_.begin (); m_it != voxel_grid_.end (); ++m_it)
  {
    Eigen::Vector3i index;
    getIndexIn3D (m_it->first, index);

    // Now dilate all of its voxels
    for (int x = -1; x <= 1; ++x)
      for (int y = -1; y <= 1; ++y)
        for (int z = -1; z <= 1; ++z)
          if (x != 0 || y != 0 || z != 0)
          {
            Eigen::Vector3i new_index;
            new_index = index + Eigen::Vector3i (x, y, z);

            std::uint64_t index_1d;
            getIndexIn1D (new_index, index_1d);
            Leaf leaf;
            new_voxel_grid[index_1d] = leaf;
          }
  }
  voxel_grid_ = new_voxel_grid;
}


/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::copyMissingFields (const PointInT &point_in,
                                                                 PointOutT &point_out) const
{
  PointOutT temp = point_out;
  copyPoint (point_in, point_out);
  point_out.x = temp.x;
  point_out.y = temp.y;
  point_out.z = temp.z;
}

#define PCL_INSTANTIATE_MovingLeastSquares(T,OutT) template class PCL_EXPORTS pcl::MovingLeastSquares<T,OutT>;
#define PCL_INSTANTIATE_MovingLeastSquaresOMP(T,OutT) template class PCL_EXPORTS pcl::MovingLeastSquaresOMP<T,OutT>;

#endif    // PCL_SURFACE_IMPL_MLS_H_
