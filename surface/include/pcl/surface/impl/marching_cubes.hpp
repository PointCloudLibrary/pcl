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

#ifndef PCL_SURFACE_IMPL_MARCHING_CUBES_H_
#define PCL_SURFACE_IMPL_MARCHING_CUBES_H_

#include <pcl/surface/marching_cubes.h>
#include <pcl/common/common.h>
#include <pcl/common/vector_average.h>
#include <pcl/Vertices.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubes<PointNT>::~MarchingCubes () = default;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::getBoundingBox ()
{
  PointNT max_pt, min_pt;
  pcl::getMinMax3D (*input_, min_pt, max_pt);

  lower_boundary_ = min_pt.getArray3fMap ();
  upper_boundary_ = max_pt.getArray3fMap ();

  const Eigen::Array3f size3_extend = 0.5f * percentage_extend_grid_ 
    * (upper_boundary_ - lower_boundary_);

  lower_boundary_ -= size3_extend;
  upper_boundary_ += size3_extend;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::interpolateEdge (Eigen::Vector3f &p1,
                                              Eigen::Vector3f &p2,
                                              float val_p1,
                                              float val_p2,
                                              Eigen::Vector3f &output)
{
  const float mu = (iso_level_ - val_p1) / (val_p2 - val_p1);
  output = p1 + mu * (p2 - p1);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::createSurface (const std::vector<float> &leaf_node,
                                            const Eigen::Vector3i &index_3d,
                                            pcl::PointCloud<PointNT> &cloud)
{
  int cubeindex = 0;
  if (leaf_node[0] < iso_level_) cubeindex |= 1;
  if (leaf_node[1] < iso_level_) cubeindex |= 2;
  if (leaf_node[2] < iso_level_) cubeindex |= 4;
  if (leaf_node[3] < iso_level_) cubeindex |= 8;
  if (leaf_node[4] < iso_level_) cubeindex |= 16;
  if (leaf_node[5] < iso_level_) cubeindex |= 32;
  if (leaf_node[6] < iso_level_) cubeindex |= 64;
  if (leaf_node[7] < iso_level_) cubeindex |= 128;

  // Cube is entirely in/out of the surface
  if (edgeTable[cubeindex] == 0)
    return;

  const Eigen::Vector3f center = lower_boundary_ 
    + size_voxel_ * index_3d.cast<float> ().array ();

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > p;
  p.resize (8);
  for (int i = 0; i < 8; ++i)
  {
    Eigen::Vector3f point = center;
    if (i & 0x4)
      point[1] = static_cast<float> (center[1] + size_voxel_[1]);

    if (i & 0x2)
      point[2] = static_cast<float> (center[2] + size_voxel_[2]);

    if ((i & 0x1) ^ ((i >> 1) & 0x1))
      point[0] = static_cast<float> (center[0] + size_voxel_[0]);

    p[i] = point;
  }

  // Find the vertices where the surface intersects the cube
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vertex_list;
  vertex_list.resize (12);
  if (edgeTable[cubeindex] & 1)
    interpolateEdge (p[0], p[1], leaf_node[0], leaf_node[1], vertex_list[0]);
  if (edgeTable[cubeindex] & 2)
    interpolateEdge (p[1], p[2], leaf_node[1], leaf_node[2], vertex_list[1]);
  if (edgeTable[cubeindex] & 4)
    interpolateEdge (p[2], p[3], leaf_node[2], leaf_node[3], vertex_list[2]);
  if (edgeTable[cubeindex] & 8)
    interpolateEdge (p[3], p[0], leaf_node[3], leaf_node[0], vertex_list[3]);
  if (edgeTable[cubeindex] & 16)
    interpolateEdge (p[4], p[5], leaf_node[4], leaf_node[5], vertex_list[4]);
  if (edgeTable[cubeindex] & 32)
    interpolateEdge (p[5], p[6], leaf_node[5], leaf_node[6], vertex_list[5]);
  if (edgeTable[cubeindex] & 64)
    interpolateEdge (p[6], p[7], leaf_node[6], leaf_node[7], vertex_list[6]);
  if (edgeTable[cubeindex] & 128)
    interpolateEdge (p[7], p[4], leaf_node[7], leaf_node[4], vertex_list[7]);
  if (edgeTable[cubeindex] & 256)
    interpolateEdge (p[0], p[4], leaf_node[0], leaf_node[4], vertex_list[8]);
  if (edgeTable[cubeindex] & 512)
    interpolateEdge (p[1], p[5], leaf_node[1], leaf_node[5], vertex_list[9]);
  if (edgeTable[cubeindex] & 1024)
    interpolateEdge (p[2], p[6], leaf_node[2], leaf_node[6], vertex_list[10]);
  if (edgeTable[cubeindex] & 2048)
    interpolateEdge (p[3], p[7], leaf_node[3], leaf_node[7], vertex_list[11]);

  // Create the triangle
  for (int i = 0; triTable[cubeindex][i] != -1; i += 3)
  {
    PointNT p1, p2, p3;
    p1.getVector3fMap () = vertex_list[triTable[cubeindex][i]];
    cloud.push_back (p1);
    p2.getVector3fMap () = vertex_list[triTable[cubeindex][i+1]];
    cloud.push_back (p2);
    p3.getVector3fMap () = vertex_list[triTable[cubeindex][i+2]];
    cloud.push_back (p3);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::getNeighborList1D (std::vector<float> &leaf,
                                                Eigen::Vector3i &index3d)
{
  leaf.resize (8);

  leaf[0] = getGridValue (index3d);
  leaf[1] = getGridValue (index3d + Eigen::Vector3i (1, 0, 0));
  leaf[2] = getGridValue (index3d + Eigen::Vector3i (1, 0, 1));
  leaf[3] = getGridValue (index3d + Eigen::Vector3i (0, 0, 1));
  leaf[4] = getGridValue (index3d + Eigen::Vector3i (0, 1, 0));
  leaf[5] = getGridValue (index3d + Eigen::Vector3i (1, 1, 0));
  leaf[6] = getGridValue (index3d + Eigen::Vector3i (1, 1, 1));
  leaf[7] = getGridValue (index3d + Eigen::Vector3i (0, 1, 1));

  for (int i = 0; i < 8; ++i)
  {
    if (std::isnan (leaf[i]))
    {
      leaf.clear ();
      break;
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> float
pcl::MarchingCubes<PointNT>::getGridValue (Eigen::Vector3i pos)
{
  /// TODO what to return?
  if (pos[0] < 0 || pos[0] >= res_x_)
    return -1.0f;
  if (pos[1] < 0 || pos[1] >= res_y_)
    return -1.0f;
  if (pos[2] < 0 || pos[2] >= res_z_)
    return -1.0f;

  return grid_[pos[0]*res_y_*res_z_ + pos[1]*res_z_ + pos[2]];
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::performReconstruction (pcl::PolygonMesh &output)
{
  pcl::PointCloud<PointNT> points;

  performReconstruction (points, output.polygons);

  pcl::toPCLPointCloud2 (points, output.cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::performReconstruction (pcl::PointCloud<PointNT> &points,
                                                    std::vector<pcl::Vertices> &polygons)
{
  if (!(iso_level_ >= 0 && iso_level_ < 1))
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Invalid iso level %f! Please use a number between 0 and 1.\n", 
        getClassName ().c_str (), iso_level_);
    points.clear ();
    polygons.clear ();
    return;
  }

  // the point cloud really generated from Marching Cubes, prev intermediate_cloud_
  pcl::PointCloud<PointNT> intermediate_cloud;

  // Create grid
  grid_ = std::vector<float> (res_x_*res_y_*res_z_, NAN);

  // Compute bounding box and voxel size
  getBoundingBox ();
  size_voxel_ = (upper_boundary_ - lower_boundary_) 
    * Eigen::Array3f (res_x_, res_y_, res_z_).inverse ();

  // Transform the point cloud into a voxel grid
  // This needs to be implemented in a child class
  voxelizeData ();

  // preallocate memory assuming a hull. suppose 6 point per voxel
  double size_reserve = std::min((double) intermediate_cloud.points.max_size (),
      2.0 * 6.0 * (double) (res_y_*res_z_ + res_x_*res_z_ + res_x_*res_y_));
  intermediate_cloud.reserve ((std::size_t) size_reserve);

  for (int x = 1; x < res_x_-1; ++x)
    for (int y = 1; y < res_y_-1; ++y)
      for (int z = 1; z < res_z_-1; ++z)
      {
        Eigen::Vector3i index_3d (x, y, z);
        std::vector<float> leaf_node;
        getNeighborList1D (leaf_node, index_3d);
        if (!leaf_node.empty ())
          createSurface (leaf_node, index_3d, intermediate_cloud);
      }

  points.swap (intermediate_cloud);

  polygons.resize (points.size () / 3);
  for (std::size_t i = 0; i < polygons.size (); ++i)
  {
    pcl::Vertices v;
    v.vertices.resize (3);
    for (int j = 0; j < 3; ++j)
      v.vertices[j] = static_cast<int> (i) * 3 + j;
    polygons[i] = v;
  }
}

#define PCL_INSTANTIATE_MarchingCubes(T) template class PCL_EXPORTS pcl::MarchingCubes<T>;

#endif    // PCL_SURFACE_IMPL_MARCHING_CUBES_H_

