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

#include <pcl/point_traits.h>
#include <pcl/surface/mls.h>
#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/geometry.h>

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
  output.points.clear ();

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
      rng_alg_.seed (static_cast<unsigned> (std::time (0)));
      float tmp = static_cast<float> (search_radius_ / 2.0f);
      boost::uniform_real<float> uniform_distrib (-tmp, tmp);
      rng_uniform_distribution_.reset (new boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > (rng_alg_, uniform_distrib));

      mls_results_.resize (1); // Need to have a reference to a single dummy result.
      
      break;
    }
    case (VOXEL_GRID_DILATION):
    case (DISTINCT_CLOUD):
      {
        mls_results_.resize (input_->size ());
        break;
      }
    default:
      {
        mls_results_.resize (1); // Need to have a reference to a single dummy result.
        break;
      }
  }

  // Perform the actual surface reconstruction
  performProcessing (output);

  if (compute_normals_)
  {
    normals_->height = 1;
    normals_->width = static_cast<uint32_t> (normals_->size ());

    for (unsigned int i = 0; i < output.size (); ++i)
    {
      typedef typename pcl::traits::fieldList<PointOutT>::type FieldList;
      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output.points[i], "normal_x", normals_->points[i].normal_x));
      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output.points[i], "normal_y", normals_->points[i].normal_y));
      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output.points[i], "normal_z", normals_->points[i].normal_z));
      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output.points[i], "curvature", normals_->points[i].curvature));
    }

  }

  // Set proper widths and heights for the clouds
  output.height = 1;
  output.width = static_cast<uint32_t> (output.size ());

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::computeMLSPointNormal (int index,
                                                                     const std::vector<int> &nn_indices,
                                                                     std::vector<float> &nn_sqr_dists,
                                                                     PointCloudOut &projected_points,
                                                                     NormalCloud &projected_points_normals,
                                                                     PointIndices &corresponding_input_indices,
                                                                     MLSResult &mls_result) const
{
  // Note: this method is const because it needs to be thread-safe
  //       (MovingLeastSquaresOMP calls it from multiple threads)

  // Compute the plane coefficients
  EIGEN_ALIGN16 Eigen::Matrix3d covariance_matrix;
  Eigen::Vector4d xyz_centroid;

  // Estimate the XYZ centroid
  pcl::compute3DCentroid (*input_, nn_indices, xyz_centroid);

  // Compute the 3x3 covariance matrix
  pcl::computeCovarianceMatrix (*input_, nn_indices, xyz_centroid, covariance_matrix);
  EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3d eigen_vector;
  Eigen::Vector4d model_coefficients;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
  model_coefficients.head<3> ().matrix () = eigen_vector;
  model_coefficients[3] = 0;
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  // Projected query point
  Eigen::Vector3d point = input_->points[index].getVector3fMap ().template cast<double> ();
  double distance = point.dot (model_coefficients.head<3> ()) + model_coefficients[3];
  point -= distance * model_coefficients.head<3> ();

  float curvature = static_cast<float> (covariance_matrix.trace ());
  // Compute the curvature surface change
  if (curvature != 0)
    curvature = fabsf (float (eigen_value / double (curvature)));


  // Get a copy of the plane normal easy access
  Eigen::Vector3d plane_normal = model_coefficients.head<3> ();
  // Vector in which the polynomial coefficients will be put
  Eigen::VectorXd c_vec;
  // Local coordinate system (Darboux frame)
  Eigen::Vector3d v_axis (0.0f, 0.0f, 0.0f), u_axis (0.0f, 0.0f, 0.0f);



  // Perform polynomial fit to update point and normal
  ////////////////////////////////////////////////////
  if (polynomial_fit_ && static_cast<int> (nn_indices.size ()) >= nr_coeff_)
  {
    // Update neighborhood, since point was projected, and computing relative
    // positions. Note updating only distances for the weights for speed
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > de_meaned (nn_indices.size ());
    for (size_t ni = 0; ni < nn_indices.size (); ++ni)
    {
      de_meaned[ni][0] = input_->points[nn_indices[ni]].x - point[0];
      de_meaned[ni][1] = input_->points[nn_indices[ni]].y - point[1];
      de_meaned[ni][2] = input_->points[nn_indices[ni]].z - point[2];
      nn_sqr_dists[ni] = static_cast<float> (de_meaned[ni].dot (de_meaned[ni]));
    }

    // Allocate matrices and vectors to hold the data used for the polynomial fit
    Eigen::VectorXd weight_vec (nn_indices.size ());
    Eigen::MatrixXd P (nr_coeff_, nn_indices.size ());
    Eigen::VectorXd f_vec (nn_indices.size ());
    Eigen::MatrixXd P_weight; // size will be (nr_coeff_, nn_indices.size ());
    Eigen::MatrixXd P_weight_Pt (nr_coeff_, nr_coeff_);

    // Get local coordinate system (Darboux frame)
    v_axis = plane_normal.unitOrthogonal ();
    u_axis = plane_normal.cross (v_axis);

    // Go through neighbors, transform them in the local coordinate system,
    // save height and the evaluation of the polynome's terms
    double u_coord, v_coord, u_pow, v_pow;
    for (size_t ni = 0; ni < nn_indices.size (); ++ni)
    {
      // (Re-)compute weights
      weight_vec (ni) = exp (-nn_sqr_dists[ni] / sqr_gauss_param_);

      // Transforming coordinates
      u_coord = de_meaned[ni].dot (u_axis);
      v_coord = de_meaned[ni].dot (v_axis);
      f_vec (ni) = de_meaned[ni].dot (plane_normal);

      // Compute the polynomial's terms at the current point
      int j = 0;
      u_pow = 1;
      for (int ui = 0; ui <= order_; ++ui)
      {
        v_pow = 1;
        for (int vi = 0; vi <= order_ - ui; ++vi)
        {
          P (j++, ni) = u_pow * v_pow;
          v_pow *= v_coord;
        }
        u_pow *= u_coord;
      }
    }

    // Computing coefficients
    P_weight = P * weight_vec.asDiagonal ();
    P_weight_Pt = P_weight * P.transpose ();
    c_vec = P_weight * f_vec;
    P_weight_Pt.llt ().solveInPlace (c_vec);
  }

  switch (upsample_method_)
  {
    case (NONE):
    {
      Eigen::Vector3d normal = plane_normal;

      if (polynomial_fit_ && static_cast<int> (nn_indices.size ()) >= nr_coeff_ && pcl_isfinite (c_vec[0]))
      {
        point += (c_vec[0] * plane_normal);

        // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec[order_+1] and c_vec[1]
        if (compute_normals_)
          normal = plane_normal - c_vec[order_ + 1] * u_axis - c_vec[1] * v_axis;
      }

      PointOutT aux;
      aux.x = static_cast<float> (point[0]);
      aux.y = static_cast<float> (point[1]);
      aux.z = static_cast<float> (point[2]);
      projected_points.push_back (aux);

      if (compute_normals_)
      {
        pcl::Normal aux_normal;
        aux_normal.normal_x = static_cast<float> (normal[0]);
        aux_normal.normal_y = static_cast<float> (normal[1]);
        aux_normal.normal_z = static_cast<float> (normal[2]);
        aux_normal.curvature = curvature;
        projected_points_normals.push_back (aux_normal);
        corresponding_input_indices.indices.push_back (index);
      }

      break;
    }

    case (SAMPLE_LOCAL_PLANE):
    {
      // Uniformly sample a circle around the query point using the radius and step parameters
      for (float u_disp = -static_cast<float> (upsampling_radius_); u_disp <= upsampling_radius_; u_disp += static_cast<float> (upsampling_step_))
        for (float v_disp = -static_cast<float> (upsampling_radius_); v_disp <= upsampling_radius_; v_disp += static_cast<float> (upsampling_step_))
          if (u_disp*u_disp + v_disp*v_disp < upsampling_radius_*upsampling_radius_)
          {
            PointOutT projected_point;
            pcl::Normal projected_normal;
            projectPointToMLSSurface (u_disp, v_disp, u_axis, v_axis, plane_normal, point,
                                      curvature, c_vec,
                                      static_cast<int> (nn_indices.size ()),
                                      projected_point, projected_normal);

            projected_points.push_back (projected_point);
            corresponding_input_indices.indices.push_back (index);
            if (compute_normals_)
              projected_points_normals.push_back (projected_normal);
          }
      break;
    }

    case (RANDOM_UNIFORM_DENSITY):
    {
      // Compute the local point density and add more samples if necessary
      int num_points_to_add = static_cast<int> (floor (desired_num_points_in_radius_ / 2.0 / static_cast<double> (nn_indices.size ())));

      // Just add the query point, because the density is good
      if (num_points_to_add <= 0)
      {
        // Just add the current point
        Eigen::Vector3d normal = plane_normal;
        if (polynomial_fit_ && static_cast<int> (nn_indices.size ()) >= nr_coeff_ && pcl_isfinite (c_vec[0]))
        {
          // Projection onto MLS surface along Darboux normal to the height at (0,0)
          point += (c_vec[0] * plane_normal);
          // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec[order_+1] and c_vec[1]
          if (compute_normals_)
            normal = plane_normal - c_vec[order_ + 1] * u_axis - c_vec[1] * v_axis;
        }
        PointOutT aux;
        aux.x = static_cast<float> (point[0]);
        aux.y = static_cast<float> (point[1]);
        aux.z = static_cast<float> (point[2]);
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
      else
      {
        // Sample the local plane
        for (int num_added = 0; num_added < num_points_to_add;)
        {
          float u_disp = (*rng_uniform_distribution_) (),
              v_disp = (*rng_uniform_distribution_) ();
          // Check if inside circle; if not, try another coin flip
          if (u_disp * u_disp + v_disp * v_disp > search_radius_ * search_radius_/4)
            continue;


          PointOutT projected_point;
          pcl::Normal projected_normal;
          projectPointToMLSSurface (u_disp, v_disp, u_axis, v_axis, plane_normal, point,
                                    curvature, c_vec,
                                    static_cast<int> (nn_indices.size ()),
                                    projected_point, projected_normal);

          projected_points.push_back (projected_point);
          corresponding_input_indices.indices.push_back (index);
          if (compute_normals_)
            projected_points_normals.push_back (projected_normal);

          num_added ++;
        }
      }
      break;
    }

    case (VOXEL_GRID_DILATION):
    case (DISTINCT_CLOUD):
    {
      // Take all point pairs and sample space between them in a grid-fashion
      // \note consider only point pairs with increasing indices
      mls_result = MLSResult (point, plane_normal, u_axis, v_axis, c_vec, static_cast<int> (nn_indices.size ()), curvature);
      break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::projectPointToMLSSurface (float &u_disp, float &v_disp,
                                                                        Eigen::Vector3d &u, Eigen::Vector3d &v,
                                                                        Eigen::Vector3d &plane_normal,
                                                                        Eigen::Vector3d &mean,
                                                                        float &curvature,
                                                                        Eigen::VectorXd &c_vec,
                                                                        int num_neighbors,
                                                                        PointOutT &result_point,
                                                                        pcl::Normal &result_normal) const
{
  double n_disp = 0.0f;
  double d_u = 0.0f, d_v = 0.0f;

  // HARDCODED 5*nr_coeff_ to guarantee that the computed polynomial had a proper point set basis
  if (polynomial_fit_ && num_neighbors >= 5*nr_coeff_ && pcl_isfinite (c_vec[0]))
  {
    // Compute the displacement along the normal using the fitted polynomial
    // and compute the partial derivatives needed for estimating the normal
    int j = 0;
    float u_pow = 1.0f, v_pow = 1.0f, u_pow_prev = 1.0f, v_pow_prev = 1.0f;
    for (int ui = 0; ui <= order_; ++ui)
    {
      v_pow = 1;
      for (int vi = 0; vi <= order_ - ui; ++vi)
      {
        // Compute displacement along normal
        n_disp += u_pow * v_pow * c_vec[j++];

        // Compute partial derivatives
        if (ui >= 1)
          d_u += c_vec[j-1] * ui * u_pow_prev * v_pow;
        if (vi >= 1)
          d_v += c_vec[j-1] * vi * u_pow * v_pow_prev;

        v_pow_prev = v_pow;
        v_pow *= v_disp;
      }
      u_pow_prev = u_pow;
      u_pow *= u_disp;
    }
  }

  result_point.x = static_cast<float> (mean[0] + u[0] * u_disp + v[0] * v_disp + plane_normal[0] * n_disp);
  result_point.y = static_cast<float> (mean[1] + u[1] * u_disp + v[1] * v_disp + plane_normal[1] * n_disp);
  result_point.z = static_cast<float> (mean[2] + u[2] * u_disp + v[2] * v_disp + plane_normal[2] * n_disp);

  Eigen::Vector3d normal = plane_normal - d_u * u - d_v * v;
  normal.normalize ();

  result_normal.normal_x = static_cast<float> (normal[0]);
  result_normal.normal_y = static_cast<float> (normal[1]);
  result_normal.normal_z = static_cast<float> (normal[2]);
  result_normal.curvature = curvature;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::performProcessing (PointCloudOut &output)
{
  // Compute the number of coefficients
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

  // Allocate enough space to hold the results of nearest neighbor searches
  // \note resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices;
  std::vector<float> nn_sqr_dists;
  
  size_t mls_result_index = 0;

  // For all points
  for (size_t cp = 0; cp < indices_->size (); ++cp)
  {
    // Get the initial estimates of point positions and their neighborhoods
    if (!searchForNeighbors ((*indices_)[cp], nn_indices, nn_sqr_dists))
      continue;


    // Check the number of nearest neighbors for normal estimation (and later
    // for polynomial fit as well)
    if (nn_indices.size () < 3)
      continue;


    PointCloudOut projected_points;
    NormalCloud projected_points_normals;
    // Get a plane approximating the local surface's tangent and project point onto it
    int index = (*indices_)[cp];
    
    if (upsample_method_ == VOXEL_GRID_DILATION || upsample_method_ == DISTINCT_CLOUD)
      mls_result_index = index; // otherwise we give it a dummy location.

    computeMLSPointNormal (index, nn_indices, nn_sqr_dists, projected_points, projected_points_normals, *corresponding_input_indices_, mls_results_[mls_result_index]);


    // Copy all information from the input cloud to the output points (not doing any interpolation)
    for (size_t pp = 0; pp < projected_points.size (); ++pp)
      copyMissingFields (input_->points[(*indices_)[cp]], projected_points[pp]);


    // Append projected points to output
    output.insert (output.end (), projected_points.begin (), projected_points.end ());
    if (compute_normals_)
      normals_->insert (normals_->end (), projected_points_normals.begin (), projected_points_normals.end ());
  }

  // Perform the distinct-cloud or voxel-grid upsampling
  performUpsampling (output);
}

//////////////////////////////////////////////////////////////////////////////////////////////
#ifdef _OPENMP
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquaresOMP<PointInT, PointOutT>::performProcessing (PointCloudOut &output)
{
  // Compute the number of coefficients
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

  // (Maximum) number of threads
  unsigned int threads = threads_ == 0 ? 1 : threads_;

  // Create temporaries for each thread in order to avoid synchronization
  typename PointCloudOut::CloudVectorType projected_points (threads);
  typename NormalCloud::CloudVectorType projected_points_normals (threads);
  std::vector<PointIndices> corresponding_input_indices (threads);

  // For all points
#pragma omp parallel for schedule (dynamic,1000) num_threads (threads)
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    // Allocate enough space to hold the results of nearest neighbor searches
    // \note resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices;
    std::vector<float> nn_sqr_dists;

    // Get the initial estimates of point positions and their neighborhoods
    if (this->searchForNeighbors ((*indices_)[cp], nn_indices, nn_sqr_dists))
    {
      // Check the number of nearest neighbors for normal estimation (and later
      // for polynomial fit as well)
      if (nn_indices.size () >= 3)
      {
        // This thread's ID (range 0 to threads-1)
        int tn = omp_get_thread_num ();

        // Size of projected points before computeMLSPointNormal () adds points
        size_t pp_size = projected_points[tn].size ();

        // Get a plane approximating the local surface's tangent and project point onto it
        int index = (*indices_)[cp];
        size_t mls_result_index = 0;
        
        if (upsample_method_ == VOXEL_GRID_DILATION || upsample_method_ == DISTINCT_CLOUD)
          mls_result_index = index; // otherwise we give it a dummy location.
        
        this->computeMLSPointNormal (index, nn_indices, nn_sqr_dists, projected_points[tn], projected_points_normals[tn], corresponding_input_indices[tn], this->mls_results_[mls_result_index]);

        // Copy all information from the input cloud to the output points (not doing any interpolation)
        for (size_t pp = pp_size; pp < projected_points[tn].size (); ++pp)
          this->copyMissingFields (input_->points[(*indices_)[cp]], projected_points[tn][pp]);
	    }
	  }
  }


  // Combine all threads' results into the output vectors
  for (unsigned int tn = 0; tn < threads; ++tn)
  {
    output.insert (output.end (), projected_points[tn].begin (), projected_points[tn].end ());
    corresponding_input_indices_->indices.insert (corresponding_input_indices_->indices.end (),
        corresponding_input_indices[tn].indices.begin (), corresponding_input_indices[tn].indices.end ());
    if (compute_normals_)
      normals_->insert (normals_->end (), projected_points_normals[tn].begin (), projected_points_normals[tn].end ());
  }

  // Perform the distinct-cloud or voxel-grid upsampling
  this->performUpsampling (output);
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::performUpsampling (PointCloudOut &output)
{
  if (upsample_method_ == DISTINCT_CLOUD)
  {
    for (size_t dp_i = 0; dp_i < distinct_cloud_->size (); ++dp_i) // dp_i = distinct_point_i
    {
      // Distinct cloud may have nan points, skip them
      if (!pcl_isfinite (distinct_cloud_->points[dp_i].x))
        continue;

      // Get 3D position of point
      //Eigen::Vector3f pos = distinct_cloud_->points[dp_i].getVector3fMap ();
      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->nearestKSearch (distinct_cloud_->points[dp_i], 1, nn_indices, nn_dists);
      int input_index = nn_indices.front ();

      // If the closest point did not have a valid MLS fitting result
      // OR if it is too far away from the sampled point
      if (mls_results_[input_index].valid == false)
        continue;

      Eigen::Vector3d add_point = distinct_cloud_->points[dp_i].getVector3fMap ().template cast<double> ();

      float u_disp = static_cast<float> ((add_point - mls_results_[input_index].mean).dot (mls_results_[input_index].u_axis)),
            v_disp = static_cast<float> ((add_point - mls_results_[input_index].mean).dot (mls_results_[input_index].v_axis));

      PointOutT result_point;
      pcl::Normal result_normal;
      projectPointToMLSSurface (u_disp, v_disp,
                                mls_results_[input_index].u_axis, mls_results_[input_index].v_axis,
                                mls_results_[input_index].plane_normal,
                                mls_results_[input_index].mean,
                                mls_results_[input_index].curvature,
                                mls_results_[input_index].c_vec,
                                mls_results_[input_index].num_neighbors,
                                result_point, result_normal);

      // Copy additional point information if available
      copyMissingFields (input_->points[input_index], result_point);

      // Store the id of the original point
      corresponding_input_indices_->indices.push_back (input_index);

      output.push_back (result_point);
      if (compute_normals_)
        normals_->push_back (result_normal);
    }
  }

  // For the voxel grid upsampling method, generate the voxel grid and dilate it
  // Then, project the newly obtained points to the MLS surface
  if (upsample_method_ == VOXEL_GRID_DILATION)
  {
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

      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree_->nearestKSearch (p, 1, nn_indices, nn_dists);
      int input_index = nn_indices.front ();

      // If the closest point did not have a valid MLS fitting result
      // OR if it is too far away from the sampled point
      if (mls_results_[input_index].valid == false)
        continue;

      Eigen::Vector3d add_point = p.getVector3fMap ().template cast<double> ();
      float u_disp = static_cast<float> ((add_point - mls_results_[input_index].mean).dot (mls_results_[input_index].u_axis)),
            v_disp = static_cast<float> ((add_point - mls_results_[input_index].mean).dot (mls_results_[input_index].v_axis));

      PointOutT result_point;
      pcl::Normal result_normal;
      projectPointToMLSSurface (u_disp, v_disp,
                                mls_results_[input_index].u_axis, mls_results_[input_index].v_axis,
                                mls_results_[input_index].plane_normal,
                                mls_results_[input_index].mean,
                                mls_results_[input_index].curvature,
                                mls_results_[input_index].c_vec,
                                mls_results_[input_index].num_neighbors,
                                result_point, result_normal);

      // Copy additional point information if available
      copyMissingFields (input_->points[input_index], result_point);

      // Store the id of the original point
      corresponding_input_indices_->indices.push_back (input_index);

      output.push_back (result_point);

      if (compute_normals_)
        normals_->push_back (result_normal);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
pcl::MovingLeastSquares<PointInT, PointOutT>::MLSResult::MLSResult (const Eigen::Vector3d &a_mean,
                                                                    const Eigen::Vector3d &a_plane_normal,
                                                                    const Eigen::Vector3d &a_u,
                                                                    const Eigen::Vector3d &a_v,
                                                                    const Eigen::VectorXd a_c_vec,
                                                                    const int a_num_neighbors,
                                                                    const float &a_curvature) :
  mean (a_mean), plane_normal (a_plane_normal), u_axis (a_u), v_axis (a_v), c_vec (a_c_vec), num_neighbors (a_num_neighbors),
  curvature (a_curvature), valid (true)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
pcl::MovingLeastSquares<PointInT, PointOutT>::MLSVoxelGrid::MLSVoxelGrid (PointCloudInConstPtr& cloud,
    IndicesPtr &indices,
    float voxel_size) :
    voxel_grid_ (), bounding_min_ (), bounding_max_ (), data_size_ (), voxel_size_ (voxel_size)
{
  pcl::getMinMax3D (*cloud, *indices, bounding_min_, bounding_max_);

  Eigen::Vector4f bounding_box_size = bounding_max_ - bounding_min_;
  double max_size = (std::max) ((std::max)(bounding_box_size.x (), bounding_box_size.y ()), bounding_box_size.z ());
  // Put initial cloud in voxel grid
  data_size_ = static_cast<uint64_t> (1.5 * max_size / voxel_size_);
  for (unsigned int i = 0; i < indices->size (); ++i)
    if (pcl_isfinite (cloud->points[(*indices)[i]].x))
    {
      Eigen::Vector3i pos;
      getCellIndex (cloud->points[(*indices)[i]].getVector3fMap (), pos);

      uint64_t index_1d;
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

            uint64_t index_1d;
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

#ifdef _OPENMP
#define PCL_INSTANTIATE_MovingLeastSquaresOMP(T,OutT) template class PCL_EXPORTS pcl::MovingLeastSquaresOMP<T,OutT>;
#endif

#endif    // PCL_SURFACE_IMPL_MLS_H_
