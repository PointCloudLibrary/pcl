/*
 * Software License Agreement (BSD License)
 * 
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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

#pragma once

#include <pcl/common/io.h>
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/search/kdtree.h> // for KdTree

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::LocalMaximum<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  Indices indices;

  output.is_dense = true;
  applyFilterIndices (indices);
  pcl::copyPointCloud<PointT> (*input_, indices, output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::LocalMaximum<PointT>::applyFilterIndices (Indices &indices)
{
  typename PointCloud::Ptr cloud_projected (new PointCloud);

  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  
  // Create the filtering object and project input into xy plane
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (input_);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Initialize the search class
  if (!searcher_)
  {
    if (input_->isOrganized ())
      searcher_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      searcher_.reset (new pcl::search::KdTree<PointT> (false));
  }
  searcher_->setInputCloud (cloud_projected);

  // The arrays to be used
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  std::vector<bool> point_is_max (indices_->size (), false);
  std::vector<bool> point_is_visited (indices_->size (), false);

  // Find all points within xy radius (i.e., a vertical cylinder) of the query
  // point, removing those that are locally maximal (i.e., highest z within the
  // cylinder)
  for (const auto& iii : (*indices_))
  {
    if (!isFinite ((*input_)[iii]))
    {
      continue;
    }

    // Points in the neighborhood of a previously identified local max, will
    // not be maximal in their own neighborhood
    if (point_is_visited[iii] && !point_is_max[iii])
    {
      continue;
    }

    // Assume the current query point is the maximum, mark as visited
    point_is_max[iii] = true;
    point_is_visited[iii] = true;

    // Perform the radius search in the projected cloud
    Indices radius_indices;
    std::vector<float> radius_dists;
    PointT p = (*cloud_projected)[iii];
    if (searcher_->radiusSearch (p, radius_, radius_indices, radius_dists) == 0)
    {
      PCL_WARN ("[pcl::%s::applyFilter] Searching for neighbors within radius %f failed.\n", getClassName ().c_str (), radius_);
      continue;
    }

    // If query point is alone, we retain it regardless
    if (radius_indices.size () == 1)
    {
        point_is_max[iii] = false;
    }

    // Check to see if a neighbor is higher than the query point
    float query_z = (*input_)[iii].z;
    for (std::size_t k = 1; k < radius_indices.size (); ++k)  // k = 1 is the first neighbor
    {
      if ((*input_)[radius_indices[k]].z > query_z)
      {
        // Query point is not the local max, no need to check others
        point_is_max[iii] = false;
        break;
      }
    }

    // If the query point was a local max, all neighbors can be marked as
    // visited, excluding them from future consideration as local maxima
    if (point_is_max[iii])
    {
      for (std::size_t k = 1; k < radius_indices.size (); ++k)  // k = 1 is the first neighbor
      {
        point_is_visited[radius_indices[k]] = true;
      }
    }

    // Points that are local maxima are passed to removed indices
    // Unless negative was set, then it's the opposite condition
    if ((!negative_ && point_is_max[iii]) || (negative_ && !point_is_max[iii]))
    {
      if (extract_removed_indices_)
      {
        (*removed_indices_)[rii++] = iii;
      }

      continue;
    }

    // Otherwise it was a normal point for output (inlier)
    indices[oii++] = iii;
  }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_LocalMaximum(T) template class PCL_EXPORTS pcl::LocalMaximum<T>;

