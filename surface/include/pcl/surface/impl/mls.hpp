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
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/geometry.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::process (PointCloudOut &output)
{
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
    PCL_ERROR ("[pcl::%s::reconstruct] Invalid search radius (%f) or Gaussian parameter (%f)!\n", getClassName ().c_str (), search_radius_, sqr_gauss_param_);
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
  tree_->setInputCloud (input_, indices_);

  // Initialize random number generator if necessary
  switch (upsample_method_)
  {
    case (RANDOM_UNIFORM_DENSITY):
    {
      boost::mt19937 *rng = new boost::mt19937 (static_cast<unsigned int>(std::time(0)));
      float tmp = static_cast<float> (search_radius_ / 2.0f);
      boost::uniform_real<float> *uniform_distrib = new boost::uniform_real<float> (-tmp, tmp);
      rng_uniform_distribution_ = new boost::variate_generator<boost::mt19937, boost::uniform_real<float> > (*rng, *uniform_distrib);
      break;
    }
    case (VOXEL_GRID_DILATION):
    {
      mls_results_.resize (input_->size ());
      break;
    }
    default:
      break;
  }

  // Perform the actual surface reconstruction
  performProcessing (output);

  if (compute_normals_)
  {
    normals_->height = 1;
    normals_->width = static_cast<uint32_t> (normals_->size ());

    // TODO!!! MODIFY TO PER-CLOUD COPYING - much faster than per-point
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
                                                                     const PointCloudIn &input,
                                                                     const std::vector<int> &nn_indices,
                                                                     std::vector<float> &nn_sqr_dists,
                                                                     PointCloudOut &projected_points,
                                                                     NormalCloud &projected_points_normals)
{
  // Compute the plane coefficients
  //pcl::computePointNormal<PointInT> (*input_, nn_indices, model_coefficients, curvature);
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  pcl::compute3DCentroid (input, nn_indices, xyz_centroid);
  //pcl::compute3DCentroid (input, nn_indices, xyz_centroid);

  pcl::computeCovarianceMatrix (input, nn_indices, xyz_centroid, covariance_matrix);
  // Compute the 3x3 covariance matrix

  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  Eigen::Vector4f model_coefficients;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
  model_coefficients.head<3> () = eigen_vector;
  model_coefficients[3] = 0;
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  // Projected query point
  Eigen::Vector3f point = input[(*indices_)[index]].getVector3fMap ();
  float distance = point.dot (model_coefficients.head<3> ()) + model_coefficients[3];
  point -= distance * model_coefficients.head<3> ();

  float curvature = covariance_matrix.trace ();
  // Compute the curvature surface change
  if (curvature != 0)
    curvature = fabsf (eigen_value / curvature);


  // Get a copy of the plane normal easy access
  Eigen::Vector3d plane_normal = model_coefficients.head<3> ().cast<double> ();
  // Vector in which the polynomial coefficients will be put
  Eigen::VectorXd c_vec;
  // Local coordinate system (Darboux frame)
  Eigen::Vector3d v (0.0f, 0.0f, 0.0f), u (0.0f, 0.0f, 0.0f);



  // Perform polynomial fit to update point and normal
  ////////////////////////////////////////////////////
  if (polynomial_fit_ && static_cast<int> (nn_indices.size ()) >= nr_coeff_)
  {
    // Update neighborhood, since point was projected, and computing relative
    // positions. Note updating only distances for the weights for speed
    std::vector<Eigen::Vector3d> de_meaned (nn_indices.size ());
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
    v = plane_normal.unitOrthogonal ();
    u = plane_normal.cross (v);

    // Go through neighbors, transform them in the local coordinate system,
    // save height and the evaluation of the polynome's terms
    double u_coord, v_coord, u_pow, v_pow;
    for (size_t ni = 0; ni < nn_indices.size (); ++ni)
    {
      // (re-)compute weights
      weight_vec (ni) = exp (-nn_sqr_dists[ni] / sqr_gauss_param_);

      // transforming coordinates
      u_coord = de_meaned[ni].dot (u);
      v_coord = de_meaned[ni].dot (v);
      f_vec (ni) = de_meaned[ni].dot (plane_normal);

      // compute the polynomial's terms at the current point
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
        point += (c_vec[0] * plane_normal).cast<float> ();

        // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec[order_+1] and c_vec[1]
        if (compute_normals_)
          normal = plane_normal - c_vec[order_ + 1] * u - c_vec[1] * v;
      }

      PointOutT aux;
      aux.x = point[0];
      aux.y = point[1];
      aux.z = point[2];
      projected_points.push_back (aux);

      if (compute_normals_)
      {
        pcl::Normal aux_normal;
        aux_normal.normal_x = static_cast<float> (normal[0]);
        aux_normal.normal_y = static_cast<float> (normal[1]);
        aux_normal.normal_z = static_cast<float> (normal[2]);
        aux_normal.curvature = curvature;
        projected_points_normals.push_back (aux_normal);
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
            projectPointToMLSSurface (u_disp, v_disp, u, v, plane_normal, curvature, point, c_vec, 
                                      static_cast<int> (nn_indices.size ()),
                                      projected_point, projected_normal);

            projected_points.push_back (projected_point);
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
          point += (c_vec[0] * plane_normal).cast<float> ();
          // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec[order_+1] and c_vec[1]
          if (compute_normals_)
            normal = plane_normal - c_vec[order_ + 1] * u - c_vec[1] * v;
        }
        PointOutT aux;
        aux.x = point[0];
        aux.y = point[1];
        aux.z = point[2];
        projected_points.push_back (aux);

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
          projectPointToMLSSurface (u_disp, v_disp, u, v, plane_normal, curvature, point, c_vec, 
                                    static_cast<int> (nn_indices.size ()),
                                    projected_point, projected_normal);

          projected_points.push_back (projected_point);
          if (compute_normals_)
            projected_points_normals.push_back (projected_normal);

          num_added ++;
        }
      }
      break;
    }

    case (VOXEL_GRID_DILATION):
    {
      // Take all point pairs and sample space between them in a grid-fashion
      // \note consider only point pairs with increasing indices
      MLSResult result (plane_normal, u, v, c_vec, static_cast<int> (nn_indices.size ()), curvature);
      mls_results_[index] = result;
      break;
    }
  }
}



template <typename PointInT, typename PointOutT> void
pcl::MovingLeastSquares<PointInT, PointOutT>::projectPointToMLSSurface (float &u_disp, float &v_disp,
                                                                        Eigen::Vector3d &u, Eigen::Vector3d &v,
                                                                        Eigen::Vector3d &plane_normal,
                                                                        float &curvature,
                                                                        Eigen::Vector3f &query_point,
                                                                        Eigen::VectorXd &c_vec,
                                                                        int num_neighbors,
                                                                        PointOutT &result_point,
                                                                        pcl::Normal &result_normal)
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

  Eigen::Vector3d normal = plane_normal - d_u * u - d_v * v;
  normal.normalize ();
  result_point.x = static_cast<float> (query_point[0] + u[0] * u_disp + v[0] * v_disp + normal[0] * n_disp);
  result_point.y = static_cast<float> (query_point[1] + u[1] * u_disp + v[1] * v_disp + normal[1] * n_disp);
  result_point.z = static_cast<float> (query_point[2] + u[2] * u_disp + v[2] * v_disp + normal[2] * n_disp);

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

  // For all points
  for (size_t cp = 0; cp < indices_->size (); ++cp)
  {
    // Get the initial estimates of point positions and their neighborhoods
    if (!searchForNeighbors (int (cp), nn_indices, nn_sqr_dists))
      continue;

    // Check the number of nearest neighbors for normal estimation (and later
    // for polynomial fit as well)
    if (nn_indices.size () < 3)
      continue;


    PointCloudOut projected_points;
    NormalCloud projected_points_normals;
    // Get a plane approximating the local surface's tangent and project point onto it
    computeMLSPointNormal (int (cp), *input_, nn_indices, nn_sqr_dists, projected_points, projected_points_normals);

    // Append projected points to output
    output.insert (output.end (), projected_points.begin (), projected_points.end ());
    if (compute_normals_)
      normals_->insert (normals_->end (), projected_points_normals.begin (), projected_points_normals.end ());
  }

 
  
  // For the voxel grid upsampling method, generate the voxel grid and dilate it
  // Then, project the newly obtained points to the MLS surface
  if (upsample_method_ == VOXEL_GRID_DILATION)
  {
    MLSVoxelGrid voxel_grid (input_, indices_, voxel_size_);
    
    for (int iteration = 0; iteration < dilation_iteration_num_; ++iteration)
      voxel_grid.dilate ();
    
    
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
      
      PointOutT result_point;
      pcl::Normal result_normal;
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
      if (compute_normals_)
        normals_->push_back (result_normal);
    }
  }
}






//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
pcl::MovingLeastSquares<PointInT, PointOutT>::MLSResult::MLSResult (Eigen::Vector3d &a_plane_normal,
                                                                    Eigen::Vector3d &a_u,
                                                                    Eigen::Vector3d &a_v,
                                                                    Eigen::VectorXd a_c_vec,
                                                                    int a_num_neighbors,
                                                                    float &a_curvature) :
  plane_normal (a_plane_normal), u (a_u), v (a_v), c_vec (a_c_vec), num_neighbors (a_num_neighbors), 
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
  BOOST_FOREACH (typename HashMap::value_type voxel, voxel_grid_)
  {
    Eigen::Vector3i index;
    getIndexIn3D (voxel.first, index);

    /// Now dilate all of its voxels
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


#define PCL_INSTANTIATE_MovingLeastSquares(T,OutT) template class PCL_EXPORTS pcl::MovingLeastSquares<T,OutT>;

#endif    // PCL_SURFACE_IMPL_MLS_H_
