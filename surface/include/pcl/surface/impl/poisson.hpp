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
 * $Id: poisson.hpp 5444 2012-03-30 19:44:43Z aichim $
 *
 */

#ifndef PCL_SURFACE_IMPL_POISSON_H_
#define PCL_SURFACE_IMPL_POISSON_H_

#include <pcl/surface/poisson.h>
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>

#include <pcl/surface/poisson/octree_poisson.h>
#include <pcl/surface/poisson/sparse_matrix.h>
#include <pcl/surface/poisson/function_data.h>
#include <pcl/surface/poisson/ppolynomial.h>
#include <pcl/surface/poisson/multi_grid_octree_data.h>

#define MEMORY_ALLOCATOR_BLOCK_SIZE 1<<12

#include <stdarg.h>
#include <string>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::Poisson<PointNT>::Poisson ()
: data_ (),
  no_reset_samples_ (false),
  no_clip_tree_ (false),
  confidence_ (false),
  manifold_ (false),
  output_polygons_ (false),
  depth_ (8),
  solver_divide_ (8),
  iso_divide_ (8),
  refine_ (3),
  kernel_depth_ (8),
  degree_ (2),
  samples_per_node_ (1.0),
  scale_ (1.25)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::Poisson<PointNT>::~Poisson ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> template <int Degree> void
pcl::Poisson<PointNT>::execute (poisson::CoredMeshData &mesh,
                                poisson::Point3D<float> &center,
                                float &scale)
{
  float isoValue = 0.0f;
  ///////////////////////////////////////
  // Fix courtesy of David Gallup      //
  poisson::TreeNodeData::UseIndex = 1; //
  ///////////////////////////////////////
  poisson::Octree<Degree> tree;
  poisson::PPolynomial<Degree> ReconstructionFunction = poisson::PPolynomial<Degree>::GaussianApproximation ();

  center.coords[0] = center.coords[1] = center.coords[2] = 0.0f;

  poisson::TreeOctNode::SetAllocator (MEMORY_ALLOCATOR_BLOCK_SIZE);

  kernel_depth_ = depth_ - 2;
//  if(KernelDepth.set){kernelDepth=KernelDepth.value;}

  tree.setFunctionData (ReconstructionFunction, depth_, 0, poisson::Real (1.0f) / poisson::Real (1 << depth_));
//  if (kernel_depth_>depth_)
//  {
//    fprintf(stderr,"KernelDepth can't be greater than Depth: %d <= %d\n",kernel_depth_,depth_);
//    return;
//  }

  tree.setTree (input_, depth_, kernel_depth_, float (samples_per_node_), scale_, center, scale, !no_reset_samples_, confidence_);

  printf ("scale after settree %f\n", scale);

  if(!no_clip_tree_)
  {
    tree.ClipTree ();
  }

  tree.finalize1 (refine_);

  tree.maxMemoryUsage = 0;
  tree.SetLaplacianWeights ();

  tree.finalize2 (refine_);

  tree.maxMemoryUsage = 0;
  tree.LaplacianMatrixIteration (solver_divide_);

  tree.maxMemoryUsage = 0;
  isoValue = tree.GetIsoValue ();

  if (iso_divide_)
    tree.GetMCIsoTriangles (isoValue, iso_divide_, &mesh, 0, 1, manifold_, output_polygons_);
  else
    tree.GetMCIsoTriangles (isoValue, &mesh, 0, 1, manifold_, output_polygons_);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::Poisson<PointNT>::performReconstruction (PolygonMesh &output)
{
  poisson::CoredVectorMeshData mesh;
  poisson::Point3D<float> center;
  float scale = 1.0f;

  switch (degree_)
  {
    case 1:
    {
      execute<1> (mesh, center, scale);
      break;
    }
    case 2:
    {
      execute<2> (mesh, center, scale);
      break;
    }
    case 3:
    {
      execute<3> (mesh, center, scale);
      break;
    }
    case 4:
    {
      execute<4> (mesh, center, scale);
      break;
    }
    case 5:
    {
      execute<5> (mesh, center, scale);
      break;
    }
    default:
    {
      PCL_ERROR (stderr, "Degree %d not supported\n", degree_);
    }
  }

  /// Write output PolygonMesh
  // write vertices
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize (int (mesh.outOfCorePointCount () + mesh.inCorePoints.size ()));
  poisson::Point3D<float> p;
  for (int i = 0; i < int (mesh.inCorePoints.size ()); i++)
  {
    p = mesh.inCorePoints[i];
    cloud.points[i].x = p.coords[0]*scale+center.coords[0];
    cloud.points[i].y = p.coords[1]*scale+center.coords[1];
    cloud.points[i].z = p.coords[2]*scale+center.coords[2];
  }
  for (int i = int (mesh.inCorePoints.size ()); i < int (mesh.outOfCorePointCount () + mesh.inCorePoints.size ()); i++)
  {
    mesh.nextOutOfCorePoint (p);
    cloud.points[i].x = p.coords[0]*scale+center.coords[0];
    cloud.points[i].y = p.coords[1]*scale+center.coords[1];
    cloud.points[i].z = p.coords[2]*scale+center.coords[2];
  }
  pcl::toROSMsg (cloud, output.cloud);
  output.polygons.resize (mesh.polygonCount ());

  // Write faces
  std::vector<poisson::CoredVertexIndex> polygon;
  for (int p_i = 0; p_i < mesh.polygonCount (); p_i++)
  {
    pcl::Vertices v;
    mesh.nextPolygon (polygon);
    v.vertices.resize (polygon.size ());

    for (int i = 0; i < static_cast<int> (polygon.size ()); ++i)
      if (polygon[i].inCore )
        v.vertices[i] = polygon[i].idx;
      else
        v.vertices[i] = polygon[i].idx + int (mesh.inCorePoints.size ());

    output.polygons[p_i] = v;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::Poisson<PointNT>::performReconstruction (pcl::PointCloud<PointNT> &points,
                                              std::vector<pcl::Vertices> &polygons)
{
  poisson::CoredVectorMeshData mesh;
  poisson::Point3D<float> center;
  float scale = 1.0f;

  switch (degree_)
  {
    case 1:
    {
      execute<1> (mesh, center, scale);
      break;
    }
    case 2:
    {
      execute<2> (mesh, center, scale);
      break;
    }
    case 3:
    {
      execute<3> (mesh, center, scale);
      break;
    }
    case 4:
    {
      execute<4> (mesh, center, scale);
      break;
    }
    case 5:
    {
      execute<5> (mesh, center, scale);
      break;
    }
    default:
    {
      PCL_ERROR (stderr, "Degree %d not supported\n", degree_);
    }
  }

  // Write output PolygonMesh
  // Write vertices
  points.points.resize (int (mesh.outOfCorePointCount () + mesh.inCorePoints.size ()));
  poisson::Point3D<float> p;
  for (int i = 0; i < int(mesh.inCorePoints.size ()); i++)
  {
    p = mesh.inCorePoints[i];
    points.points[i].x = p.coords[0]*scale+center.coords[0];
    points.points[i].y = p.coords[1]*scale+center.coords[1];
    points.points[i].z = p.coords[2]*scale+center.coords[2];
  }
  for (int i = int(mesh.inCorePoints.size()); i < int (mesh.outOfCorePointCount() + mesh.inCorePoints.size ()); i++)
  {
    mesh.nextOutOfCorePoint (p);
    points.points[i].x = p.coords[0]*scale+center.coords[0];
    points.points[i].y = p.coords[1]*scale+center.coords[1];
    points.points[i].z = p.coords[2]*scale+center.coords[2];
  }



  polygons.resize (mesh.polygonCount ());

  // write faces
  std::vector<poisson::CoredVertexIndex> polygon;
  for (int p_i = 0; p_i < mesh.polygonCount (); p_i++)
  {
    pcl::Vertices v;
    mesh.nextPolygon (polygon);
    v.vertices.resize (polygon.size ());

    for (int i = 0; i < static_cast<int> (polygon.size ()); ++i)
      if (polygon[i].inCore )
        v.vertices[i] = polygon[i].idx;
      else
        v.vertices[i] = polygon[i].idx + int (mesh.inCorePoints.size ());

    polygons[p_i] = v;
  }
}


#define PCL_INSTANTIATE_Poisson(T) template class PCL_EXPORTS pcl::Poisson<T>;

#endif    // PCL_SURFACE_IMPL_POISSON_H_

