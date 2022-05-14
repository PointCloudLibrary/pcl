/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_FILTERS_CONVOLUTION_3D_IMPL_HPP
#define PCL_FILTERS_CONVOLUTION_3D_IMPL_HPP

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

///////////////////////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
  namespace filters
  {
    template <typename PointT>
    class ConvolvingKernel<PointT, pcl::Normal>
    {
      void
      makeInfinite (pcl::Normal& n)
      {
        n.normal_x = n.normal_y = n.normal_z = std::numeric_limits<float>::quiet_NaN ();
      }
    };

    template <typename PointT> class
    ConvolvingKernel<PointT, pcl::PointXY>
    {
      void
      makeInfinite (pcl::PointXY& p)
      {
        p.x = p.y = std::numeric_limits<float>::quiet_NaN ();
      }
    };
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> bool
pcl::filters::GaussianKernel<PointInT, PointOutT>::initCompute ()
{
  if (sigma_ == 0)
  {
    PCL_ERROR ("Sigma is not set or equal to 0!\n", sigma_);
    return (false);
  }
  sigma_sqr_ = sigma_ * sigma_;

  if (sigma_coefficient_)
  {
    if ((*sigma_coefficient_) > 6 || (*sigma_coefficient_) < 3)
    {
      PCL_ERROR ("Sigma coefficient (%f) out of [3..6]!\n", (*sigma_coefficient_));
      return (false);
    }
    else
      threshold_ = (*sigma_coefficient_) * (*sigma_coefficient_) * sigma_sqr_;
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> PointOutT
pcl::filters::GaussianKernel<PointInT, PointOutT>::operator() (const Indices& indices,
                                                               const std::vector<float>& distances)
{
  using namespace pcl::common;
  PointOutT result;
  float total_weight = 0;
  std::vector<float>::const_iterator dist_it = distances.begin ();

  for (Indices::const_iterator idx_it = indices.begin ();
       idx_it != indices.end ();
       ++idx_it, ++dist_it)
  {
    if (*dist_it <= threshold_ && isFinite ((*input_) [*idx_it]))
    {
      float weight = std::exp (-0.5f * (*dist_it) / sigma_sqr_);
      result += weight * (*input_) [*idx_it];
      total_weight += weight;
    }
  }
  if (total_weight != 0)
    result /= total_weight;
  else
    makeInfinite (result);

  return (result);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> PointOutT
pcl::filters::GaussianKernelRGB<PointInT, PointOutT>::operator() (const Indices& indices, const std::vector<float>& distances)
{
  using namespace pcl::common;
  PointOutT result;
  float total_weight = 0;
  float r = 0, g = 0, b = 0;
  std::vector<float>::const_iterator dist_it = distances.begin ();

  for (Indices::const_iterator idx_it = indices.begin ();
       idx_it != indices.end ();
       ++idx_it, ++dist_it)
  {
    if (*dist_it <= threshold_ && isFinite ((*input_) [*idx_it]))
    {
      float weight = std::exp (-0.5f * (*dist_it) / sigma_sqr_);
      result.x += weight * (*input_) [*idx_it].x;
      result.y += weight * (*input_) [*idx_it].y;
      result.z += weight * (*input_) [*idx_it].z;
      r += weight * static_cast<float> ((*input_) [*idx_it].r);
      g += weight * static_cast<float> ((*input_) [*idx_it].g);
      b += weight * static_cast<float> ((*input_) [*idx_it].b);
      total_weight += weight;
    }
  }
  if (total_weight != 0)
  {
    total_weight = 1.f/total_weight;
    r*= total_weight; g*= total_weight; b*= total_weight;
    result.x*= total_weight; result.y*= total_weight; result.z*= total_weight;
    result.r = static_cast<std::uint8_t> (r);
    result.g = static_cast<std::uint8_t> (g);
    result.b = static_cast<std::uint8_t> (b);
  }
  else
    makeInfinite (result);

  return (result);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename KernelT>
pcl::filters::Convolution3D<PointInT, PointOutT, KernelT>::Convolution3D ()
  : PCLBase <PointInT> ()
  , surface_ ()
  , tree_ ()
  , search_radius_ (0)
{}

///////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename KernelT> bool
pcl::filters::Convolution3D<PointInT, PointOutT, KernelT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL_ERROR ("[pcl::filters::Convlution3D::initCompute] init failed!\n");
    return (false);
  }
  // Initialize the spatial locator
  if (!tree_)
  {
    if (input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointInT> (false));
  }
  // If no search surface has been defined, use the input dataset as the search surface itself
  if (!surface_)
    surface_ = input_;
  // Send the surface dataset to the spatial locator
  tree_->setInputCloud (surface_);
  // Do a fast check to see if the search parameters are well defined
  if (search_radius_ <= 0.0)
  {
    PCL_ERROR ("[pcl::filters::Convlution3D::initCompute] search radius (%f) must be > 0\n",
               search_radius_);
    return (false);
  }
  // Make sure the provided kernel implements the required interface
  if (dynamic_cast<ConvolvingKernel<PointInT, PointOutT>* > (&kernel_) == 0)
  {
    PCL_ERROR ("[pcl::filters::Convlution3D::initCompute] init failed : ");
    PCL_ERROR ("kernel_ must implement ConvolvingKernel interface\n!");
    return (false);
  }
  kernel_.setInputCloud (surface_);
  // Initialize convolving kernel
  if (!kernel_.initCompute ())
  {
    PCL_ERROR ("[pcl::filters::Convlution3D::initCompute] kernel initialization failed!\n");
    return (false);
  }
  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename KernelT> void
pcl::filters::Convolution3D<PointInT, PointOutT, KernelT>::convolve (PointCloudOut& output)
{
  if (!initCompute ())
  {
    PCL_ERROR ("[pcl::filters::Convlution3D::convolve] init failed!\n");
    return;
  }
  output.resize (surface_->size ());
  output.width = surface_->width;
  output.height = surface_->height;
  output.is_dense = surface_->is_dense;
  Indices nn_indices;
  std::vector<float> nn_distances;

#pragma omp parallel for \
  default(none) \
  shared(output) \
  firstprivate(nn_indices, nn_distances) \
  num_threads(threads_)
  for (std::int64_t point_idx = 0; point_idx < static_cast<std::int64_t> (surface_->size ()); ++point_idx)
  {
    const PointInT& point_in = surface_->points [point_idx];
    PointOutT& point_out = output [point_idx];
    if (isFinite (point_in) &&
        tree_->radiusSearch (point_in, search_radius_, nn_indices, nn_distances))
    {
      point_out = kernel_ (nn_indices, nn_distances);
    }
    else
    {
      kernel_.makeInfinite (point_out);
      output.is_dense = false;
    }
  }
}

#endif
