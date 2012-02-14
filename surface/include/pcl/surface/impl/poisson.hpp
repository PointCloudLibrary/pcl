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
 */

#ifndef PCL_SURFACE_IMPL_POISSON_H_
#define PCL_SURFACE_IMPL_POISSON_H_

#include "pcl/surface/poisson.h"
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "pcl/surface/poisson/octree.h"
#include "pcl/surface/poisson/sparse_matrix.h"
#include "pcl/surface/poisson/function_data.h"
#include "pcl/surface/poisson/ppolynomial.h"
#include "pcl/surface/poisson/multi_grid_octree_data.h"

#define MEMORY_ALLOCATOR_BLOCK_SIZE 1<<12

#include <stdarg.h>
#include <string>

using namespace pcl::surface::poisson;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::surface::Poisson<PointNT>::Poisson ()
  : no_reset_samples_(false),
    no_clip_tree_(false),
    confidence_(false),
    manifold_(false),
    depth_(10),
    solver_divide_(8),
    iso_divide_(8),
    refine_(3),
    kernel_depth_(8),
    degree_ (2),
    samples_per_node_(1.0),
    scale_(1.25)
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::surface::Poisson<PointNT>::~Poisson ()
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> template <int Degree> void
pcl::surface::Poisson<PointNT>::execute (CoredMeshData &mesh,
                                         Point3D<float> &center)
{
  float scale = 1;
  //////////////////////////////////
  // Fix courtesy of David Gallup //
  // TreeNodeData::UseIndex = 1;  //
  //////////////////////////////////
  Octree<Degree> tree;
  PPolynomial<Degree> ReconstructionFunction=PPolynomial<Degree>::GaussianApproximation();

  center.coords[0]=center.coords[1]=center.coords[2]=0;

  TreeOctNode::SetAllocator(MEMORY_ALLOCATOR_BLOCK_SIZE);

  int kernel_depth=depth_-2;
  if(kernel_depth_){kernel_depth=kernel_depth_;}

  tree.setFunctionData(ReconstructionFunction,depth_,0,float(1.0)/(1<<depth_));
  if (kernel_depth > depth_)
  {
    PCL_ERROR ("KernelDepth can't be greater than Depth: %d <= %d\n",kernel_depth,depth_);
    return;
  }


  tree.setTree (depth_, kernel_depth, float(samples_per_node_), scale_, center, scale, !no_reset_samples_, confidence_, input_);

  if(!no_clip_tree_)
    tree.ClipTree();

  tree.finalize1(refine_);


  tree.SetLaplacianWeights();

  tree.finalize2(refine_);

  tree.LaplacianMatrixIteration(solver_divide_);

  float isoValue=tree.GetIsoValue();

  if (iso_divide_)
    tree.GetMCIsoTriangles (isoValue, iso_divide_, &mesh, 0, 1, manifold_);
  else
    tree.GetMCIsoTriangles (isoValue, &mesh, 0, 1, manifold_);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::surface::Poisson<PointNT>::performReconstruction (PolygonMesh &output)
{
  CoredVectorMeshData mesh;
  Point3D<float> center;

  switch(degree_)
  {
    case 1:
      execute<1> (mesh, center);
      break;
    case 2:
      execute<2> (mesh, center);
      break;
    case 3:
      execute<3> (mesh, center);
      break;
    case 4:
      execute<4> (mesh, center);
      break;
    case 5:
      execute<5> (mesh, center);
      break;
    default:
      PCL_ERROR (stderr,"Degree %d not supported\n", degree_);
  }

  /// Write output PolygonMesh
  float scale = 1;
   // write vertices
   pcl::PointCloud < pcl::PointXYZ > cloud;
   cloud.points.resize (int (mesh.outOfCorePointCount()+mesh.inCorePoints.size ()) );
   Point3D<float> p;
   for (int i = 0; i < int(mesh.inCorePoints.size()); i++)
   {
     p = mesh.inCorePoints[i];
     cloud.points[i].x = p.coords[0]*scale+center.coords[0];
     cloud.points[i].y = p.coords[1]*scale+center.coords[1];
     cloud.points[i].z = p.coords[2]*scale+center.coords[2];
   }
   for (int i = int(mesh.inCorePoints.size()); i < int (mesh.outOfCorePointCount() + mesh.inCorePoints.size ()); i++)
   {
     mesh.nextOutOfCorePoint(p);
     cloud.points[i].x = p.coords[0]*scale+center.coords[0];
     cloud.points[i].y = p.coords[1]*scale+center.coords[1];
     cloud.points[i].z = p.coords[2]*scale+center.coords[2];
   }
   pcl::toROSMsg (cloud, output.cloud);
   output.polygons.resize (mesh.triangleCount ());

   // write faces
   TriangleIndex tIndex;
   int inCoreFlag;
   for (int i = 0; i < mesh.triangleCount(); i++)
   {
     // create and fill a struct that the ply code can handle
     pcl::Vertices v;
     v.vertices.resize (3);

     mesh.nextTriangle(tIndex,inCoreFlag);
     if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0])) { tIndex.idx[0] += int(mesh.inCorePoints.size()); }
     if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1])) { tIndex.idx[1] += int(mesh.inCorePoints.size()); }
     if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2])) { tIndex.idx[2] += int(mesh.inCorePoints.size()); }
     for (int j = 0; j < 3; j++) {v.vertices[j] = tIndex.idx[j];}
     output.polygons[i] = v;
   }  // for, write faces
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::surface::Poisson<PointNT>::performReconstruction (pcl::PointCloud<PointNT> &points,
                                                       std::vector<pcl::Vertices> &polygons)
{
  CoredVectorMeshData mesh;
  Point3D<float> center;

  switch(degree_)
  {
    case 1:
      execute<1> (mesh, center);
      break;
    case 2:
      execute<2> (mesh, center);
      break;
    case 3:
      execute<3> (mesh, center);
      break;
    case 4:
      execute<4> (mesh, center);
      break;
    case 5:
      execute<5> (mesh, center);
      break;
    default:
      PCL_ERROR (stderr,"Degree %d not supported\n", degree_);
  }

  /// Write output PolygonMesh
  float scale = 1;
   // write vertices
   points.points.resize (int (mesh.outOfCorePointCount()+mesh.inCorePoints.size ()) );
   Point3D<float> p;
   for (int i = 0; i < int(mesh.inCorePoints.size()); i++)
   {
     p = mesh.inCorePoints[i];
     points.points[i].x = p.coords[0]*scale+center.coords[0];
     points.points[i].y = p.coords[1]*scale+center.coords[1];
     points.points[i].z = p.coords[2]*scale+center.coords[2];
   }
   for (int i = int(mesh.inCorePoints.size()); i < int (mesh.outOfCorePointCount() + mesh.inCorePoints.size ()); i++)
   {
     mesh.nextOutOfCorePoint(p);
     points.points[i].x = p.coords[0]*scale+center.coords[0];
     points.points[i].y = p.coords[1]*scale+center.coords[1];
     points.points[i].z = p.coords[2]*scale+center.coords[2];
   }

   polygons.resize (mesh.triangleCount ());

   // write faces
   TriangleIndex tIndex;
   int inCoreFlag;
   for (int i = 0; i < mesh.triangleCount(); i++)
   {
     // create and fill a struct that the ply code can handle
     pcl::Vertices v;
     v.vertices.resize (3);

     mesh.nextTriangle(tIndex,inCoreFlag);
     if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0])) { tIndex.idx[0] += int(mesh.inCorePoints.size()); }
     if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1])) { tIndex.idx[1] += int(mesh.inCorePoints.size()); }
     if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2])) { tIndex.idx[2] += int(mesh.inCorePoints.size()); }
     for (int j = 0; j < 3; j++) {v.vertices[j] = tIndex.idx[j];}
     polygons[i] = v;
   }  // for, write faces

}


#define PCL_INSTANTIATE_Poisson(T) template class PCL_EXPORTS pcl::surface::Poisson<T>;

#endif    // PCL_SURFACE_IMPL_POISSON_H_

