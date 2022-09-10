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
 * $Id$
 *
 */

#ifndef PCL_SURFACE_IMPL_POISSON_H_
#define PCL_SURFACE_IMPL_POISSON_H_

#include <pcl/surface/poisson.h>
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>

#include <pcl/surface/3rdparty/poisson4/octree_poisson.h>
#include <pcl/surface/3rdparty/poisson4/sparse_matrix.h>
#include <pcl/surface/3rdparty/poisson4/function_data.h>
#include <pcl/surface/3rdparty/poisson4/ppolynomial.h>
#include <pcl/surface/3rdparty/poisson4/multi_grid_octree_data.h>
#include <pcl/surface/3rdparty/poisson4/geometry.h>

#define MEMORY_ALLOCATOR_BLOCK_SIZE 1<<12

#include <cstdarg>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::Poisson<PointNT>::Poisson ()
  : depth_ (8)
  , min_depth_ (5)
  , point_weight_ (4)
  , scale_ (1.1f)
  , solver_divide_ (8)
  , iso_divide_ (8)
  , samples_per_node_ (1.0)
  , confidence_ (false)
  , output_polygons_ (false)
  , no_reset_samples_ (false)
  , no_clip_tree_ (false)
  , manifold_ (true)
  , refine_ (3)
  , kernel_depth_ (8)
  , degree_ (2)
  , non_adaptive_weights_ (false)
  , show_residual_ (false)
  , min_iterations_ (8)
  , solver_accuracy_ (1e-3f)
  , threads_(1)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::Poisson<PointNT>::~Poisson () = default;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::Poisson<PointNT>::setThreads (int threads)
{
  if (threads == 0)
#ifdef _OPENMP
    threads_ = omp_get_num_procs();
#else
    threads_ = 1;
#endif
  else
    threads_ = threads;
}
      
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> template <int Degree> void
pcl::Poisson<PointNT>::execute (poisson::CoredVectorMeshData &mesh,
                                poisson::Point3D<float> &center,
                                float &scale)
{
  pcl::poisson::Real iso_value = 0;
  poisson::TreeNodeData::UseIndex = 1;
  poisson::Octree<Degree> tree;

  
  tree.threads = threads_;
  center.coords[0] = center.coords[1] = center.coords[2] = 0;


  if (solver_divide_ < min_depth_)
  {
    PCL_WARN ("[pcl::Poisson] solver_divide_ must be at least as large as min_depth_: %d >= %d\n", solver_divide_, min_depth_);
    solver_divide_ = min_depth_;
  }
  if (iso_divide_< min_depth_)
  {
    PCL_WARN ("[pcl::Poisson] iso_divide_ must be at least as large as min_depth_: %d >= %d\n", iso_divide_, min_depth_);
    iso_divide_ = min_depth_;
  }

  pcl::poisson::TreeOctNode::SetAllocator (MEMORY_ALLOCATOR_BLOCK_SIZE);

  kernel_depth_ = depth_ - 2;

  tree.setBSplineData (depth_, pcl::poisson::Real (1.0 / (1 << depth_)), true);

  tree.maxMemoryUsage = 0;


  int point_count = tree.template setTree<PointNT> (input_, depth_, min_depth_, kernel_depth_, samples_per_node_,
                                                    scale_, center, scale, confidence_, point_weight_, !non_adaptive_weights_);

  tree.ClipTree ();
  tree.finalize ();
  tree.RefineBoundary (iso_divide_);

  PCL_DEBUG ("Input Points: %d\n" , point_count );
  PCL_DEBUG ("Leaves/Nodes: %d/%d\n" , tree.tree.leaves() , tree.tree.nodes() );

  tree.maxMemoryUsage = 0;
  tree.SetLaplacianConstraints ();

  tree.maxMemoryUsage = 0;
  tree.LaplacianMatrixIteration (solver_divide_, show_residual_, min_iterations_, solver_accuracy_);

  iso_value = tree.GetIsoValue ();

  tree.GetMCIsoTriangles (iso_value, iso_divide_, &mesh, 0, 1, manifold_, output_polygons_);
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

  // Write output PolygonMesh
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.resize (int (mesh.outOfCorePointCount () + mesh.inCorePoints.size ()));
  poisson::Point3D<float> p;
  for (int i = 0; i < int (mesh.inCorePoints.size ()); i++)
  {
    p = mesh.inCorePoints[i];
    cloud[i].x = p.coords[0]*scale+center.coords[0];
    cloud[i].y = p.coords[1]*scale+center.coords[1];
    cloud[i].z = p.coords[2]*scale+center.coords[2];
  }
  for (int i = int (mesh.inCorePoints.size ()); i < int (mesh.outOfCorePointCount () + mesh.inCorePoints.size ()); i++)
  {
    mesh.nextOutOfCorePoint (p);
    cloud[i].x = p.coords[0]*scale+center.coords[0];
    cloud[i].y = p.coords[1]*scale+center.coords[1];
    cloud[i].z = p.coords[2]*scale+center.coords[2];
  }
  pcl::toPCLPointCloud2 (cloud, output.cloud);
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
  points.resize (int (mesh.outOfCorePointCount () + mesh.inCorePoints.size ()));
  poisson::Point3D<float> p;
  for (int i = 0; i < int(mesh.inCorePoints.size ()); i++)
  {
    p = mesh.inCorePoints[i];
    points[i].x = p.coords[0]*scale+center.coords[0];
    points[i].y = p.coords[1]*scale+center.coords[1];
    points[i].z = p.coords[2]*scale+center.coords[2];
  }
  for (int i = int(mesh.inCorePoints.size()); i < int (mesh.outOfCorePointCount() + mesh.inCorePoints.size ()); i++)
  {
    mesh.nextOutOfCorePoint (p);
    points[i].x = p.coords[0]*scale+center.coords[0];
    points[i].y = p.coords[1]*scale+center.coords[1];
    points[i].z = p.coords[2]*scale+center.coords[2];
  }

  polygons.resize (mesh.polygonCount ());

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

    polygons[p_i] = v;
  }
}


#define PCL_INSTANTIATE_Poisson(T) template class PCL_EXPORTS pcl::Poisson<T>;

#endif    // PCL_SURFACE_IMPL_POISSON_H_

