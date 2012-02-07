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

#include "pcl/surface/mls.h"
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename NormalOutT> void
pcl::MovingLeastSquares<PointInT, NormalOutT>::reconstruct (PointCloudIn &output)
{
  // check if normals have to be computed/saved
  if (normals_)
  {
    // Copy the header
    normals_->header = input_->header;
    // Clear the fields in case the method exits before computation
    normals_->width = normals_->height = 0;
    normals_->points.clear ();
  }

  // Copy the header
  output.header = input_->header;

  if (search_radius_ <= 0 || sqr_gauss_param_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::reconstruct] Invalid search radius (%f) or Gaussian parameter (%f)!\n", getClassName ().c_str (), search_radius_, sqr_gauss_param_);
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  if (!initCompute ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

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
  if (upsample_method_ == UNIFORM_DENSITY)
  {
    boost::mt19937 *rng = new boost::mt19937 (static_cast<unsigned int>(std::time(0)));
    boost::uniform_real<float> *uniform_distrib = new boost::uniform_real<float> (-search_radius_/2, search_radius_/2);
    rng_uniform_distribution_ = new boost::variate_generator<boost::mt19937, boost::uniform_real<float> > (*rng, *uniform_distrib);
  }

  // Perform the actual surface reconstruction
  performReconstruction (output);

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename NormalOutT> void
pcl::MovingLeastSquares<PointInT, NormalOutT>::computeMLSPointNormal (int index,
                                                                      const PointCloudIn &input,
                                                                      const std::vector<int> &nn_indices,
                                                                      std::vector<float> &nn_sqr_dists,
                                                                      PointCloudIn &projected_points,
                                                                      NormalCloudOut &projected_points_normals)
{
  // Compute the plane coefficients
  //pcl::computePointNormal<PointInT> (*input_, nn_indices, model_coefficients, curvature);
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  pcl::compute3DCentroid (input, nn_indices, xyz_centroid);

  // Compute the 3x3 covariance matrix
  pcl::computeCovarianceMatrix (input, nn_indices, xyz_centroid, covariance_matrix);

  // Get the plane normal
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value = -1;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  Eigen::Vector4f model_coefficients;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
  model_coefficients.head<3> () = eigen_vector;
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  // Projected query point
  Eigen::Vector3f point = input[index].getVector3fMap ();
  float distance = point.dot (model_coefficients.head<3> ()) + model_coefficients[3];
  point -= distance * model_coefficients.head<3> ();

  float curvature = covariance_matrix.trace ();
  // Compute the curvature surface change
  if (curvature != 0)
    curvature = fabs (eigen_value / curvature);


  // Get a copy of the plane normal easy access
  Eigen::Vector3d plane_normal = model_coefficients.head<3> ().cast<double> ();
  // Vector in which the polynomial coefficients will be put
  Eigen::VectorXd c_vec;
  // Local coordinate system (Darboux frame)
  Eigen::Vector3d v (0.0f, 0.0f, 0.0f), u (0.0f, 0.0f, 0.0f);



  // Perform polynomial fit to update point and normal
  ////////////////////////////////////////////////////
  if (polynomial_fit_ && (int)nn_indices.size () >= nr_coeff_)
  {
    // Update neighborhood, since point was projected, and computing relative
    // positions. Note updating only distances for the weights for speed
    std::vector<Eigen::Vector3d> de_meaned (nn_indices.size ());
    for (size_t ni = 0; ni < nn_indices.size (); ++ni)
    {
      de_meaned[ni][0] = input_->points[nn_indices[ni]].x - point[0];
      de_meaned[ni][1] = input_->points[nn_indices[ni]].y - point[1];
      de_meaned[ni][2] = input_->points[nn_indices[ni]].z - point[2];
      nn_sqr_dists[ni] = de_meaned[ni].dot (de_meaned[ni]);
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

      if (polynomial_fit_ && (int)nn_indices.size () >= nr_coeff_ && pcl_isfinite (c_vec[0]))
      {
        point += (c_vec[0] * plane_normal).cast<float> ();

        // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec[order_+1] and c_vec[1]
        if (normals_)
          normal = plane_normal - c_vec[order_ + 1] * u - c_vec[1] * v;
      }

      PointInT aux;
      aux.x = point[0];
      aux.y = point[1];
      aux.z = point[2];
      projected_points.push_back (aux);

      NormalOutT aux_normal;
      aux_normal.normal_x = normal[0];
      aux_normal.normal_y = normal[1];
      aux_normal.normal_z = normal[2];
      aux_normal.curvature = curvature;
      projected_points_normals.push_back (aux_normal);

      break;
    }

    case (SAMPLE_LOCAL_PLANE):
    {
      // Uniformly sample a circle around the query point using the radius and step parameters
      for (float u_disp = -upsampling_radius_; u_disp <= upsampling_radius_; u_disp += upsampling_step_)
        for (float v_disp = -upsampling_radius_; v_disp <= upsampling_radius_; v_disp += upsampling_step_)
          if (u_disp*u_disp + v_disp*v_disp < upsampling_radius_*upsampling_radius_)
          {
            PointInT projected_point;
            NormalOutT projected_normal;
            projectPointToMLSSurface (u_disp, v_disp, u, v, plane_normal, curvature, point, c_vec, nn_indices.size (),
                                      projected_point, projected_normal);

            projected_points.push_back (projected_point);
            projected_points_normals.push_back (projected_normal);
          }
      break;
    }

    case (UNIFORM_DENSITY):
    {
      // Compute the local point density and add more samples if necessary
      int num_points_to_add = (int) floor (desired_num_points_in_radius_/2 / nn_indices.size ());

      // Just add the query point, because the density is good
      if (num_points_to_add <= 0)
      {
        // Just add the current point
        Eigen::Vector3d normal = plane_normal;
        if (polynomial_fit_ && (int)nn_indices.size () >= nr_coeff_ && pcl_isfinite (c_vec[0]))
        {
          // Projection onto MLS surface along Darboux normal to the height at (0,0)
          point += (c_vec[0] * plane_normal).cast<float> ();
          // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec[order_+1] and c_vec[1]
          if (normals_)
            normal = plane_normal - c_vec[order_ + 1] * u - c_vec[1] * v;
        }
        PointInT aux;
        aux.x = point[0];
        aux.y = point[1];
        aux.z = point[2];
        projected_points.push_back (aux);

        NormalOutT aux_normal;
        aux_normal.normal_x = normal[0];
        aux_normal.normal_y = normal[1];
        aux_normal.normal_z = normal[2];
        aux_normal.curvature = curvature;
        projected_points_normals.push_back (aux_normal);
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


          PointInT projected_point;
          NormalOutT projected_normal;
          projectPointToMLSSurface (u_disp, v_disp, u, v, plane_normal, curvature, point, c_vec, nn_indices.size (),
                                    projected_point, projected_normal);

          projected_points.push_back (projected_point);
          projected_points_normals.push_back (projected_normal);

          num_added ++;
        }
      }
      break;
    }

    case (FILL_HOLES):
    {
      // Take all point pairs and sample space between them in a grid-fashion
      // \note consider only point pairs with increasing indices
      for (size_t p1_i = 0; p1_i < nn_indices.size (); ++p1_i)
        for (size_t p2_i = 0; p2_i < nn_indices.size (); ++p2_i)
          if (nn_indices[p1_i] < nn_indices[p2_i])
          {
            PointInT p1 = input_->points[nn_indices[p1_i]],
                     p2 = input_->points[nn_indices[p2_i]];


            // Sample in between with filling_step_size_ steps
            int x_dir, y_dir, z_dir;
            do {
              x_dir = (p1.x - p2.x >= filling_step_size_) ? 1 : ((p2.x - p1.x >= filling_step_size_) ? -1 : 0);
              y_dir = (p1.y - p2.y >= filling_step_size_) ? 1 : ((p2.y - p1.y >= filling_step_size_) ? -1 : 0);
              z_dir = (p1.z - p2.z >= filling_step_size_) ? 1 : ((p2.z - p1.z >= filling_step_size_) ? -1 : 0);

              Eigen::Vector3f hole_point (p1.x + x_dir * filling_step_size_,
                                          p1.y + y_dir * filling_step_size_,
                                          p1.z + z_dir * filling_step_size_);
              float u_disp = (hole_point - point).dot (u.cast<float> ()),
                    v_disp = (hole_point - point).dot (v.cast<float> ());

              PointInT projected_point;
              NormalOutT projected_normal;
              projectPointToMLSSurface (u_disp, v_disp, u, v, plane_normal, curvature, point, c_vec, nn_indices.size (),
                                        projected_point, projected_normal);

              projected_points.push_back (projected_point);
              projected_points_normals.push_back (projected_normal);
            } while ((x_dir != 0) || (y_dir != 0) || (z_dir != 0));
          }


      // And then add the query point itself
      Eigen::Vector3d normal = plane_normal;

      if (polynomial_fit_ && (int)nn_indices.size () >= nr_coeff_ && pcl_isfinite (c_vec[0]))
      {
        point += (c_vec[0] * plane_normal).cast<float> ();

        // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec[order_+1] and c_vec[1]
        if (normals_)
          normal = plane_normal - c_vec[order_ + 1] * u - c_vec[1] * v;
      }

      PointInT aux;
      aux.x = point[0];
      aux.y = point[1];
      aux.z = point[2];
      projected_points.push_back (aux);

      NormalOutT aux_normal;
      aux_normal.normal_x = normal[0];
      aux_normal.normal_y = normal[1];
      aux_normal.normal_z = normal[2];
      aux_normal.curvature = curvature;
      projected_points_normals.push_back (aux_normal);

      break;
    }
  }
}



template <typename PointInT, typename NormalOutT> void
pcl::MovingLeastSquares<PointInT, NormalOutT>::projectPointToMLSSurface (float &u_disp, float &v_disp,
                                                                         Eigen::Vector3d &u, Eigen::Vector3d &v,
                                                                         Eigen::Vector3d &plane_normal,
                                                                         float &curvature,
                                                                         Eigen::Vector3f &query_point,
                                                                         Eigen::VectorXd &c_vec,
                                                                         int num_neighbors,
                                                                         PointInT &result_point,
                                                                         NormalOutT &result_normal)
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
  result_point.x = query_point[0] + u[0] * u_disp + v[0] * v_disp + normal[0] * n_disp;
  result_point.y = query_point[1] + u[1] * u_disp + v[1] * v_disp + normal[1] * n_disp;
  result_point.z = query_point[2] + u[2] * u_disp + v[2] * v_disp + normal[2] * n_disp;

  result_normal.normal_x = normal[0];
  result_normal.normal_y = normal[1];
  result_normal.normal_z = normal[2];
  result_normal.curvature = curvature;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename NormalOutT> void
pcl::MovingLeastSquares<PointInT, NormalOutT>::performReconstruction (PointCloudIn &output)
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
    if (!searchForNeighbors ((*indices_)[cp], nn_indices, nn_sqr_dists))
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

  // Set proper widths and heights for the clouds
  if (upsample_method_ == NONE && fake_indices_)
  {
    normals_->width = input_->width;
    normals_->height = input_->height;
    output.width = input_->width;
    output.height = input_->height;
  }
  else
  {
    normals_->height = 1;
    normals_->width = normals_->size ();
    output.height = 1;
    output.width = output.size ();
  }
}

#define PCL_INSTANTIATE_MovingLeastSquares(T,OutT) template class PCL_EXPORTS pcl::MovingLeastSquares<T,OutT>;

#endif    // PCL_SURFACE_IMPL_MLS_H_
