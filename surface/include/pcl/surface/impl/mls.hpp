/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: mls.hpp 36043 2011-02-17 23:59:49Z marton $
 *
 */

#ifndef PCL_SURFACE_IMPL_MLS_H_
#define PCL_SURFACE_IMPL_MLS_H_

#include "pcl/surface/mls.h"

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
  
  if (!initCompute ()) 
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Check if a space search locator was given
  if (!tree_)
  {
    ROS_ERROR ("[pcl::%s::compute] No spatial search method was given!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Send the surface dataset to the spatial locator
  tree_->setInputCloud (input_, indices_);

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    output.width    = indices_->size ();
    output.height   = 1;
  }
  else
  {
    output.width    = input_->width;
    output.height   = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Resize the output normal dataset
  if (normals_)
  {
    normals_->points.resize (output.points.size ());
    normals_->width    = output.width;
    normals_->height   = output.height;
    normals_->is_dense = output.is_dense;
  }

  // Perform the actual surface reconstruction
  performReconstruction (output);

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename NormalOutT> void
pcl::MovingLeastSquares<PointInT, NormalOutT>::performReconstruction (PointCloudIn &output)
{
  if (search_radius_ <= 0 || sqr_gauss_param_ <= 0)
  {
    ROS_ERROR ("[pcl::%s::performReconstruction] Invalid search radius (%f) or Gaussian parameter (%f)!", getClassName ().c_str (), search_radius_, sqr_gauss_param_);
    output.width = output.height = 0;
    output.points.clear ();
    if (normals_)
    {
      normals_->width = normals_->height = 0;
      normals_->points.clear ();
    }
    return;
  }

  // Compute the number of coefficients
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

  // Allocate enough space to hold the results of nearest neighbor searches
  // \note resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices;
  std::vector<float> nn_sqr_dists;
  
  // Use original point positions for fitting
  // \note no up/down/adapting-sampling or hole filling possible like this
  output.points.resize (indices_->size ());
  pcl::copyPointCloud (*input_, *indices_, output);

  // For all points
  for (size_t cp = 0; cp < indices_->size (); ++cp)
  {
    // Get the initial estimates of point positions and their neighborhoods
    ///////////////////////////////////////////////////////////////////////

    // Search for the nearest neighbors
    if (!searchForNeighbors ((*indices_)[cp], nn_indices, nn_sqr_dists))
    {
      if (normals_)
        normals_->points[cp].normal[0] = normals_->points[cp].normal[1] = normals_->points[cp].normal[2] = normals_->points[cp].curvature = std::numeric_limits<float>::quiet_NaN ();
      continue;
    }

    // Check the number of nearest neighbors for normal estimation (and later
    // for polynomial fit as well)
    int k = nn_indices.size ();
    if (k < 3)
      continue;

    // Get a plane approximating the local surface's tangent and project point onto it
    //////////////////////////////////////////////////////////////////////////////////

    // Compute the plane coefficients
    Eigen::Vector4f model_coefficients;
    float curvature;
    pcl::computePointNormal<PointInT> (*input_, nn_indices, model_coefficients, curvature);

    // Projected point
    Eigen::Vector3f point = output.points[cp].getVector3fMap ();
    float distance = point.dot (model_coefficients.head<3> ()) + model_coefficients[3];
    point -= distance * model_coefficients.head<3> ();

    // Perform polynomial fit to update point and normal
    ////////////////////////////////////////////////////
    if (polynomial_fit_ && k >= nr_coeff_)
    {
      // For easy change between float and double
      typedef Eigen::Vector3d Evector3;
      typedef Eigen::VectorXd Evector;
      typedef Eigen::MatrixXd Ematrix;
      // Get a copy of the plane normal easy access
      Evector3 plane_normal = model_coefficients.head<3> ().cast<double> ();

      // Update neighborhood, since point was projected, and computing relative
      // positions. Note updating only distances for the weights for speed
      std::vector<Evector3> de_meaned (k);
      for (int ni = 0; ni < k; ++ni)
      {
        de_meaned[ni][0] = input_->points[nn_indices[ni]].x - point[0];
        de_meaned[ni][1] = input_->points[nn_indices[ni]].y - point[1];
        de_meaned[ni][2] = input_->points[nn_indices[ni]].z - point[2];
        nn_sqr_dists[ni] = de_meaned[ni].dot (de_meaned[ni]);
      }

      // Allocate matrices and vectors to hold the data used for the polynomial
      // fit
      Evector weight_vec_ (k);
      Ematrix P_ (nr_coeff_, k);
      Evector f_vec_ (k);
      Evector c_vec_;
      Ematrix P_weight_; // size will be (nr_coeff_, k);
      Ematrix P_weight_Pt_ (nr_coeff_, nr_coeff_);

      // Get local coordinate system (Darboux frame)
      Evector3 v = plane_normal.unitOrthogonal ();
      Evector3 u = plane_normal.cross (v);

      // Go through neighbors, transform them in the local coordinate system,
      // save height and the evaluation of the polynome's terms
      double u_coord, v_coord, u_pow, v_pow;
      for (int ni = 0; ni < k; ++ni)
      {
        // (re-)compute weights
        weight_vec_ (ni) = exp (-nn_sqr_dists[ni] / sqr_gauss_param_);

        // transforming coordinates
        u_coord = de_meaned[ni].dot (u);
        v_coord = de_meaned[ni].dot (v);
        f_vec_(ni) = de_meaned[ni].dot (plane_normal);

        // compute the polynomial's terms at the current point
        int j = 0;
        u_pow = 1;
        for (int ui = 0; ui <= order_; ++ui)
        {
          v_pow = 1;
          for (int vi = 0; vi <= order_ - ui; ++vi)
          {
            P_ (j++, ni) = u_pow * v_pow;
            v_pow *= v_coord;
          }
          u_pow *= u_coord;
        }
      }

      // Computing coefficients
      P_weight_ = P_ * weight_vec_.asDiagonal ();
      P_weight_Pt_ = P_weight_ * P_.transpose ();
      c_vec_ = P_weight_ * f_vec_;
      P_weight_Pt_.llt ().solveInPlace (c_vec_);

      // Projection onto MLS surface along Darboux normal to the height at (0,0)
      if (!pcl_isfinite (c_vec_[0]))
      {
        point += (c_vec_[0] * plane_normal).cast<float> ();

        // Compute tangent vectors using the partial derivates evaluated at (0,0) which is c_vec_[order_+1] and c_vec_[1]
        if (normals_)
        {
          Evector3 n_a = u + plane_normal * c_vec_[order_ + 1];
          Evector3 n_b = v + plane_normal * c_vec_[1];
          model_coefficients.head<3> () = n_a.cross (n_b).cast<float> ();
          model_coefficients.head<3> ().normalize ();
        }
      }
    }

    // Save results to output cloud
    ///////////////////////////////
    output.points[cp].x = point[0];
    output.points[cp].y = point[1];
    output.points[cp].z = point[2];
    if (normals_)
    {
      normals_->points[cp].normal[0] = model_coefficients[0];
      normals_->points[cp].normal[1] = model_coefficients[1];
      normals_->points[cp].normal[2] = model_coefficients[2];
      normals_->points[cp].curvature = curvature;
    }
  }
}

#define PCL_INSTANTIATE_MovingLeastSquares(T,OutT) template class pcl::MovingLeastSquares<T,OutT>;

#endif    // PCL_SURFACE_IMPL_MLS_H_

