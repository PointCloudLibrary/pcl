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
 */


#ifndef PCL_KEYPOINT_IMPL_H_
#define PCL_KEYPOINT_IMPL_H_

#include <pcl/console/print.h> // for PCL_ERROR

#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/search/kdtree.h> // for KdTree

namespace pcl
{

template <typename PointInT, typename PointOutT> bool
Keypoint<PointInT, PointOutT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
    return (false);

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
  if (search_radius_ != 0.0)
  {
    if (k_ != 0)
    {
      PCL_ERROR ("[pcl::%s::initCompute] Both radius (%f) and K (%d) defined! Set one of them to zero first and then re-run compute ().\n", getClassName ().c_str (), search_radius_, k_);
      return (false);
    }

    // Use the radiusSearch () function
    search_parameter_ = search_radius_;
    if (surface_ == input_)       // if the two surfaces are the same
    {
      // Declare the search locator definition
      search_method_ = [this] (pcl::index_t index, double radius, pcl::Indices &k_indices, std::vector<float> &k_distances)
      {
        return tree_->radiusSearch (index, radius, k_indices, k_distances, 0);
      };
    }
    else
    {
      // Declare the search locator definition
      search_method_surface_ = [this] (const PointCloudIn &cloud, pcl::index_t index, double radius, pcl::Indices &k_indices, std::vector<float> &k_distances)
      {
        return tree_->radiusSearch (cloud, index, radius, k_indices, k_distances, 0);
      };
    }
  }
  else
  {
    if (k_ != 0)         // Use the nearestKSearch () function
    {
      search_parameter_ = k_;
      if (surface_ == input_)       // if the two surfaces are the same
      {
        // Declare the search locator definition
        search_method_ = [this] (pcl::index_t index, int k, pcl::Indices &k_indices, std::vector<float> &k_distances)
        {
          return tree_->nearestKSearch (index, k, k_indices, k_distances);
        };
      }
      else
      {
        // Declare the search locator definition
        search_method_surface_ = [this] (const PointCloudIn &cloud, pcl::index_t index, int k, pcl::Indices &k_indices, std::vector<float> &k_distances)
        {
          return tree_->nearestKSearch (cloud, index, k, k_indices, k_distances);
        };
      }
    }
    else
    {
      PCL_ERROR ("[pcl::%s::initCompute] Neither radius nor K defined! Set one of them to a positive number first and then re-run compute ().\n", getClassName ().c_str ());
      return (false);
    }
  }

  keypoints_indices_.reset (new pcl::PointIndices);
  keypoints_indices_->indices.reserve (input_->size ());

  return (true);
}


template <typename PointInT, typename PointOutT> inline void
Keypoint<PointInT, PointOutT>::compute (PointCloudOut &output)
{
  if (!initCompute ())
  {
    PCL_ERROR ("[pcl::%s::compute] initCompute failed!\n", getClassName ().c_str ());
    return;
  }

  // Perform the actual computation
  detectKeypoints (output);

  deinitCompute ();

  // Reset the surface
  if (input_ == surface_)
    surface_.reset ();
}

} // namespace pcl

#endif  //#ifndef PCL_KEYPOINT_IMPL_H_

