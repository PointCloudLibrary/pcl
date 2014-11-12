/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */


#ifndef SURFACE_H_
#define SURFACE_H_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>

#include "typedefs.h"


class Mesh
{
public:
  Mesh () : points (new PointCloud) {}
  PointCloudPtr points;
  std::vector<pcl::Vertices> faces;
};

typedef boost::shared_ptr<Mesh> MeshPtr;

PointCloudPtr
smoothPointCloud (const PointCloudPtr & input, float radius, int polynomial_order)
{
  PointCloudPtr output (new PointCloud);
  return (output);
}

SurfaceElementsPtr
computeSurfaceElements (const PointCloudPtr & input, float radius, int polynomial_order)
{
  SurfaceElementsPtr surfels (new SurfaceElements);
  return (surfels);
}

MeshPtr
computeConvexHull (const PointCloudPtr & input)
{
  MeshPtr output (new Mesh);
  return (output);
}


MeshPtr
computeConcaveHull (const PointCloudPtr & input, float alpha)
{
  MeshPtr output (new Mesh);
  return (output);
}

pcl::PolygonMesh::Ptr
greedyTriangulation (const SurfaceElementsPtr & surfels, float radius, float mu, int max_nearest_neighbors, 
                     float max_surface_angle, float min_angle, float max_angle)

{
  pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);
  return (output);
}


pcl::PolygonMesh::Ptr
marchingCubesTriangulation (const SurfaceElementsPtr & surfels, float leaf_size, float iso_level)
{
  pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);
  return (output);
}

#endif
