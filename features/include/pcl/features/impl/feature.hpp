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
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_FEATURE_H_
#define PCL_FEATURES_IMPL_FEATURE_H_

#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/search/organized.h> // for OrganizedNeighbor


namespace pcl
{

inline void
solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                      const Eigen::Vector4f &point,
                      Eigen::Vector4f &plane_parameters, float &curvature)
{
  solvePlaneParameters (covariance_matrix, plane_parameters [0], plane_parameters [1], plane_parameters [2], curvature);

  plane_parameters[3] = 0;
  // Hessian form (D = nc . p_plane (centroid here) + p)
  plane_parameters[3] = -1 * plane_parameters.dot (point);
}


inline void
solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                      float &nx, float &ny, float &nz, float &curvature)
{
  // Avoid getting hung on Eigen's optimizers
//  for (int i = 0; i < 9; ++i)
//    if (!std::isfinite (covariance_matrix.coeff (i)))
//    {
//      //PCL_WARN ("[pcl::solvePlaneParameters] Covariance matrix has NaN/Inf values!\n");
//      nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
//      return;
//    }
  // Extract the smallest eigenvalue and its eigenvector
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);

  nx = eigen_vector [0];
  ny = eigen_vector [1];
  nz = eigen_vector [2];

  // Compute the curvature surface change
  float eig_sum = covariance_matrix.coeff (0) + covariance_matrix.coeff (4) + covariance_matrix.coeff (8);
  if (eig_sum != 0)
    curvature = std::abs (eigen_value / eig_sum);
  else
    curvature = 0;
}


template <typename PointInT, typename PointOutT> bool
Feature<PointInT, PointOutT>::initCompute ()
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
    if (surface_->isOrganized () && input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointInT> (false));
  }

  if (tree_->getInputCloud () != surface_) // Make sure the tree searches the surface
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
      return (false);
    }
    else // Use the radiusSearch () function
    {
      search_parameter_ = search_radius_;
      // Declare the search locator definition
      search_method_surface_ = [this] (const PointCloudIn &cloud, int index, double radius,
                                       pcl::Indices &k_indices, std::vector<float> &k_distances)
      {
        return tree_->radiusSearch (cloud, index, radius, k_indices, k_distances, 0);
      };
    }
  }
  else
  {
    if (k_ != 0) // Use the nearestKSearch () function
    {
      search_parameter_ = k_;
      // Declare the search locator definition
      search_method_surface_ = [this] (const PointCloudIn &cloud, int index, int k, pcl::Indices &k_indices,
                                       std::vector<float> &k_distances)
      {
        return tree_->nearestKSearch (cloud, index, k, k_indices, k_distances);
      };
    }
    else
    {
      PCL_ERROR ("[pcl::%s::compute] Neither radius nor K defined! ", getClassName ().c_str ());
      PCL_ERROR ("Set one of them to a positive number first and then re-run compute ().\n");
      // Cleanup
      deinitCompute ();
      return (false);
    }
  }
  return (true);
}


template <typename PointInT, typename PointOutT> bool
Feature<PointInT, PointOutT>::deinitCompute ()
{
  // Reset the surface
  if (fake_surface_)
  {
    surface_.reset ();
    fake_surface_ = false;
  }
  return (true);
}


template <typename PointInT, typename PointOutT> void
Feature<PointInT, PointOutT>::compute (PointCloudOut &output)
{
  if (!initCompute ())
  {
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  if (output.size () != indices_->size ())
    output.resize (indices_->size ());

  // Check if the output will be computed for all points or only a subset
  // If the input width or height are not set, set output width as size
  if (indices_->size () != input_->points.size () || input_->width * input_->height == 0)
  {
    output.width = indices_->size ();
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
}


template <typename PointInT, typename PointNT, typename PointOutT> bool
FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ()
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
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  // Check if the size of normals is the same as the size of the surface
  if (normals_->points.size () != surface_->points.size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] ", getClassName ().c_str ());
    PCL_ERROR("The number of points in the surface dataset (%zu) differs from ",
              static_cast<std::size_t>(surface_->points.size()));
    PCL_ERROR("the number of points in the dataset containing the normals (%zu)!\n",
              static_cast<std::size_t>(normals_->points.size()));
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  return (true);
}


template <typename PointInT, typename PointLT, typename PointOutT> bool
FeatureFromLabels<PointInT, PointLT, PointOutT>::initCompute ()
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // Check if input normals are set
  if (!labels_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] No input dataset containing labels was given!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  // Check if the size of normals is the same as the size of the surface
  if (labels_->points.size () != surface_->points.size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] The number of points in the input dataset differs from the number of points in the dataset containing the labels!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  return (true);
}


template <typename PointInT, typename PointRFT> bool
FeatureWithLocalReferenceFrames<PointInT, PointRFT>::initLocalReferenceFrames (const std::size_t& indices_size,
                                                                               const LRFEstimationPtr& lrf_estimation)
{
  if (frames_never_defined_)
    frames_.reset ();

  // Check if input frames are set
  if (!frames_)
  {
    if (!lrf_estimation)
    {
      PCL_ERROR ("[initLocalReferenceFrames] No input dataset containing reference frames was given!\n");
      return (false);
    } else
    {
      //PCL_WARN ("[initLocalReferenceFrames] No input dataset containing reference frames was given! Proceed using default\n");
      PointCloudLRFPtr default_frames (new PointCloudLRF());
      lrf_estimation->compute (*default_frames);
      frames_ = default_frames;
    }
  }

  // Check if the size of frames is the same as the size of the input cloud
  if (frames_->points.size () != indices_size)
  {
    if (!lrf_estimation)
    {
      PCL_ERROR ("[initLocalReferenceFrames] The number of points in the input dataset differs from the number of points in the dataset containing the reference frames!\n");
      return (false);
    } else
    {
      //PCL_WARN ("[initLocalReferenceFrames] The number of points in the input dataset differs from the number of points in the dataset containing the reference frames! Proceed using default\n");
      PointCloudLRFPtr default_frames (new PointCloudLRF());
      lrf_estimation->compute (*default_frames);
      frames_ = default_frames;
    }
  }

  return (true);
}

} // namespace pcl

#endif  //#ifndef PCL_FEATURES_IMPL_FEATURE_H_

