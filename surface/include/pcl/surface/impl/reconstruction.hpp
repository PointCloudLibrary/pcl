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
 * $Id$
 *
 */

#ifndef PCL_SURFACE_RECONSTRUCTION_IMPL_H_
#define PCL_SURFACE_RECONSTRUCTION_IMPL_H_
#include <pcl/search/pcl_search.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceReconstruction<PointInT>::reconstruct (pcl::PolygonMesh &output)
{
  // Copy the header
  output.header = input_->header;

  if (!initCompute ()) 
  {
    output.cloud.width = output.cloud.height = 0;
    output.cloud.data.clear ();
    output.polygons.clear ();
    return;
  }

  // Check if a space search locator was given
  if (check_tree_)
  {
    if (!tree_)
    {
      if (input_->isOrganized ())
        tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
      else
        tree_.reset (new pcl::search::KdTree<PointInT> (false));
    }

    // Send the surface dataset to the spatial locator
    tree_->setInputCloud (input_, indices_);
  }

  // Set up the output dataset
  pcl::toPCLPointCloud2 (*input_, output.cloud); /// NOTE: passing in boost shared pointer with * as const& should be OK here
  output.polygons.clear ();
  output.polygons.reserve (2*indices_->size ()); /// NOTE: usually the number of triangles is around twice the number of vertices
  // Perform the actual surface reconstruction
  performReconstruction (output);

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceReconstruction<PointInT>::reconstruct (pcl::PointCloud<PointInT> &points,
                                                   std::vector<pcl::Vertices> &polygons)
{
  // Copy the header
  points.header = input_->header;

  if (!initCompute ()) 
  {
    points.width = points.height = 0;
    points.clear ();
    polygons.clear ();
    return;
  }

  // Check if a space search locator was given
  if (check_tree_)
  {
    if (!tree_)
    {
      if (input_->isOrganized ())
        tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
      else
        tree_.reset (new pcl::search::KdTree<PointInT> (false));
    }

    // Send the surface dataset to the spatial locator
    tree_->setInputCloud (input_, indices_);
  }

  // Set up the output dataset
  polygons.clear ();
  polygons.reserve (2 * indices_->size ()); /// NOTE: usually the number of triangles is around twice the number of vertices
  // Perform the actual surface reconstruction
  performReconstruction (points, polygons);

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::MeshConstruction<PointInT>::reconstruct (pcl::PolygonMesh &output)
{
  // Copy the header
  output.header = input_->header;

  if (!initCompute ()) 
  {
    output.cloud.width = output.cloud.height = 1;
    output.cloud.data.clear ();
    output.polygons.clear ();
    return;
  }

  // Check if a space search locator was given
  if (check_tree_)
  {
    if (!tree_)
    {
      if (input_->isOrganized ())
        tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
      else
        tree_.reset (new pcl::search::KdTree<PointInT> (false));
    }

    // Send the surface dataset to the spatial locator
    tree_->setInputCloud (input_, indices_);
  }

  // Set up the output dataset
  pcl::toPCLPointCloud2 (*input_, output.cloud); /// NOTE: passing in boost shared pointer with * as const& should be OK here
  //  output.polygons.clear ();
  //  output.polygons.reserve (2*indices_->size ()); /// NOTE: usually the number of triangles is around twice the number of vertices
  // Perform the actual surface reconstruction
  performReconstruction (output);

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::MeshConstruction<PointInT>::reconstruct (std::vector<pcl::Vertices> &polygons)
{
  if (!initCompute ()) 
  {
    polygons.clear ();
    return;
  }

  // Check if a space search locator was given
  if (check_tree_)
  {
    if (!tree_)
    {
      if (input_->isOrganized ())
        tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
      else
        tree_.reset (new pcl::search::KdTree<PointInT> (false));
    }

    // Send the surface dataset to the spatial locator
    tree_->setInputCloud (input_, indices_);
  }

  // Set up the output dataset
  //polygons.clear ();
  //polygons.reserve (2 * indices_->size ()); /// NOTE: usually the number of triangles is around twice the number of vertices
  // Perform the actual surface reconstruction
  performReconstruction (polygons);

  deinitCompute ();
}


#endif  // PCL_SURFACE_RECONSTRUCTION_IMPL_H_

