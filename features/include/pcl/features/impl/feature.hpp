/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_IMPL_FEATURE_H_
#define PCL_FEATURES_IMPL_FEATURE_H_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/organized_data.h>

//////////////////////////////////////////////////////////////////////////////////////////////
inline void
pcl::solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix, 
                           const Eigen::Vector4f &point,
                           Eigen::Vector4f &plane_parameters, float &curvature)
{
  // Avoid getting hung on Eigen's optimizers
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (!pcl_isfinite (covariance_matrix (i, j)))
      {
        //PCL_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!\n");
        plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
        curvature = std::numeric_limits<float>::quiet_NaN ();
        return;
      }

  // Extract the eigenvalues and eigenvectors
  //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
  //EIGEN_ALIGN16 Eigen::Vector3f eigen_values  = ei_symm.eigenvalues ();
  //EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors = ei_symm.eigenvectors ();
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
  // Note: Remember to take care of the eigen_vectors ordering
  //float norm = 1.0 / eigen_vectors.col (0).norm ();

  //plane_parameters[0] = eigen_vectors (0, 0) * norm;
  //plane_parameters[1] = eigen_vectors (1, 0) * norm;
  //plane_parameters[2] = eigen_vectors (2, 0) * norm;

  // The normalization is not necessary, since the eigenvectors from libeigen are already normalized
  plane_parameters[0] = eigen_vectors (0, 0);
  plane_parameters[1] = eigen_vectors (1, 0);
  plane_parameters[2] = eigen_vectors (2, 0);
  plane_parameters[3] = 0;

  // Hessian form (D = nc . p_plane (centroid here) + p)
  plane_parameters[3] = -1 * plane_parameters.dot (point);

  // Compute the curvature surface change
  float eig_sum = eigen_values.sum ();
  if (eig_sum != 0)
    curvature = fabs ( eigen_values[0] / eig_sum );
  else
    curvature = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
inline void
pcl::solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                           float &nx, float &ny, float &nz, float &curvature)
{
  // Avoid getting hung on Eigen's optimizers
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (!pcl_isfinite (covariance_matrix (i, j)))
      {
        //PCL_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!\n");
        nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
        return;
      }
  // Extract the eigenvalues and eigenvectors
  //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
  //EIGEN_ALIGN16 Eigen::Vector3f eigen_values  = ei_symm.eigenvalues ();
  //EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors = ei_symm.eigenvectors ();
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
  // Note: Remember to take care of the eigen_vectors ordering
  //float norm = 1.0 / eigen_vectors.col (0).norm ();

  //nx = eigen_vectors (0, 0) * norm;
  //ny = eigen_vectors (1, 0) * norm;
  //nz = eigen_vectors (2, 0) * norm;
  
  // The normalization is not necessary, since the eigenvectors from libeigen are already normalized
  nx = eigen_vectors (0, 0);
  ny = eigen_vectors (1, 0);
  nz = eigen_vectors (2, 0);

  // Compute the curvature surface change
  float eig_sum = eigen_values.sum ();
  if (eig_sum != 0)
    curvature = fabs ( eigen_values[0] / eig_sum );
  else
    curvature = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::Feature<PointInT, PointOutT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::compute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return (false);
  }

  // If no search surface has been defined, use the input dataset as the search surface itself
  if (!surface_)
  {
    fake_surface_ = true;
    surface_ = input_;
  }

  // Check if a space search locator was given
  if (!tree_)
  {
    if (surface_->isOrganized ())
      tree_.reset (new pcl::OrganizedDataIndex<PointInT> ());
    else
      tree_.reset (new pcl::KdTreeFLANN<PointInT> (false));
  }
  // Send the surface dataset to the spatial locator
  tree_->setInputCloud (surface_);

  // Do a fast check to see if the search parameters are well defined
  if (search_radius_ != 0.0)
  {
    if (k_ != 0)
    {
      PCL_ERROR ("[pcl::%s::compute] ", getClassName ().c_str ());
      PCL_ERROR ("Both radius (%f) and K (%d) defined! ", search_radius_, k_);
      PCL_ERROR ("Set one of them to zero first and then re-run compute ().\n");
      // Cleanup
      deinitCompute ();
      // Reset the surface
      if (fake_surface_)
      {
        surface_.reset ();
        fake_surface_ = false;
      }
      return (false);
    }
    else // Use the radiusSearch () function
    {
      search_parameter_ = search_radius_;
      if (surface_ == input_) // if the two surfaces are the same
      {
        // Declare the search locator definition
        int (KdTree::*radiusSearch)(int index, double radius, std::vector<int> &k_indices,
                                    std::vector<float> &k_distances, int max_nn) const = &KdTree::radiusSearch;
        search_method_ = boost::bind (radiusSearch, boost::ref (tree_), _1, _2, _3, _4, INT_MAX);
      }

      // Declare the search locator definition
      int (KdTree::*radiusSearchSurface)(const PointCloudIn &cloud, int index, double radius, 
                                         std::vector<int> &k_indices, std::vector<float> &k_distances, 
                                         int max_nn) const = &KdTree::radiusSearch;
      search_method_surface_ = boost::bind (radiusSearchSurface, boost::ref (tree_), _1, _2, _3, _4, _5, INT_MAX);
    }
  }
  else
  {
    if (k_ != 0) // Use the nearestKSearch () function
    {
      search_parameter_ = k_;
      if (surface_ == input_) // if the two surfaces are the same
      {
        // Declare the search locator definition
        int (KdTree::*nearestKSearch)(int index, int k, std::vector<int> &k_indices, 
                                      std::vector<float> &k_distances) = &KdTree::nearestKSearch;
        search_method_ = boost::bind (nearestKSearch, boost::ref (tree_), _1, _2, _3, _4);
      }
      // Declare the search locator definition
      int (KdTree::*nearestKSearchSurface)(const PointCloudIn &cloud, int index, int k, std::vector<int> &k_indices, 
                                           std::vector<float> &k_distances) = &KdTree::nearestKSearch;
      search_method_surface_ = boost::bind (nearestKSearchSurface, boost::ref (tree_), _1, _2, _3, _4, _5);
    }
    else
    {
      PCL_ERROR ("[pcl::%s::compute] Neither radius nor K defined! ", getClassName ().c_str ());
      PCL_ERROR ("Set one of them to a positive number first and then re-run compute ().\n");
      // Cleanup
      deinitCompute ();
      // Reset the surface
      if (fake_surface_)
      {
        surface_.reset ();
        fake_surface_ = false;
      }
      return (false);
    }
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::Feature<PointInT, PointOutT>::compute (PointCloudOut &output)
{
  if (!initCompute ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    output.width = (int) indices_->size ();
    output.height = 1;
  }
  else
  {
    output.width = input_->width;
    output.height = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Perform the actual feature computation
  computeFeature (output);

  deinitCompute ();

  // Reset the surface
  if (fake_surface_)
  {
    surface_.reset ();
    fake_surface_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ()
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // Check if input normals are set
  if (!normals_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] No input dataset containing normals was given!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute();
    return (false);
  }

  // Check if the size of normals is the same as the size of the surface
  if (normals_->points.size () != surface_->points.size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] ", getClassName ().c_str ());
    PCL_ERROR ("The number of points in the input dataset differs from ");
    PCL_ERROR ("the number of points in the dataset containing the normals!\n");
    Feature<PointInT, PointOutT>::deinitCompute();
    return (false);
  }

  return (true);
}

#endif  //#ifndef PCL_FEATURES_IMPL_FEATURE_H_

