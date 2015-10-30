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
#include <pcl/kdtree/kdtree_flann.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubes<PointNT>::MarchingCubes () 
: min_p_ (), max_p_ (), percentage_extend_grid_ (), iso_level_ ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT>
pcl::MarchingCubes<PointNT>::~MarchingCubes ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::getBoundingBox ()
{
  pcl::getMinMax3D (*input_, min_p_, max_p_);

  min_p_ -= (max_p_ - min_p_) * percentage_extend_grid_/2;
  max_p_ += (max_p_ - min_p_) * percentage_extend_grid_/2;

  Eigen::Vector4f bounding_box_size = max_p_ - min_p_;

  bounding_box_size = max_p_ - min_p_;
  PCL_DEBUG ("[pcl::MarchingCubesHoppe::getBoundingBox] Size of Bounding Box is [%f, %f, %f]\n",
             bounding_box_size.x (), bounding_box_size.y (), bounding_box_size.z ());
  double max_size =
      (std::max) ((std::max)(bounding_box_size.x (), bounding_box_size.y ()),
          bounding_box_size.z ());
  (void)max_size;
  // ????
  //  data_size_ = static_cast<uint64_t> (max_size / leaf_size_);
  PCL_DEBUG ("[pcl::MarchingCubesHoppe::getBoundingBox] Lower left point is [%f, %f, %f]\n",
             min_p_.x (), min_p_.y (), min_p_.z ());
  PCL_DEBUG ("[pcl::MarchingCubesHoppe::getBoundingBox] Upper left point is [%f, %f, %f]\n",
             max_p_.x (), max_p_.y (), max_p_.z ());
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::interpolateEdge (Eigen::Vector3f &p1,
                                              Eigen::Vector3f &p2,
                                              float val_p1,
                                              float val_p2,
                                              Eigen::Vector3f &output)
{
  float mu = (iso_level_ - val_p1) / (val_p2-val_p1);
  output = p1 + mu * (p2 - p1);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::createSurface (std::vector<float> &leaf_node,
                                            Eigen::Vector3i &index_3d,
                                            pcl::PointCloud<PointNT> &cloud)
{
  int cubeindex = 0;
  Eigen::Vector3f vertex_list[12];
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

  //Eigen::Vector4f index_3df (index_3d[0], index_3d[1], index_3d[2], 0.0f);
  Eigen::Vector3f center;// TODO coeff wise product = min_p_ + Eigen::Vector4f (1.0f/res_x_, 1.0f/res_y_, 1.0f/res_z_) * index_3df * (max_p_ - min_p_);
  center[0] = min_p_[0] + (max_p_[0] - min_p_[0]) * float (index_3d[0]) / float (res_x_);
  center[1] = min_p_[1] + (max_p_[1] - min_p_[1]) * float (index_3d[1]) / float (res_y_);
  center[2] = min_p_[2] + (max_p_[2] - min_p_[2]) * float (index_3d[2]) / float (res_z_);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > p;
  p.resize (8);
  for (int i = 0; i < 8; ++i)
  {
    Eigen::Vector3f point = center;
    if(i & 0x4)
      point[1] = static_cast<float> (center[1] + (max_p_[1] - min_p_[1]) / float (res_y_));

    if(i & 0x2)
      point[2] = static_cast<float> (center[2] + (max_p_[2] - min_p_[2]) / float (res_z_));

    if((i & 0x1) ^ ((i >> 1) & 0x1))
      point[0] = static_cast<float> (center[0] + (max_p_[0] - min_p_[0]) / float (res_x_));

    p[i] = point;
  }


  // Find the vertices where the surface intersects the cube
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
  for (int i = 0; triTable[cubeindex][i] != -1; i+=3)
  {
    PointNT p1,p2,p3;
    p1.x = vertex_list[triTable[cubeindex][i  ]][0];
    p1.y = vertex_list[triTable[cubeindex][i  ]][1];
    p1.z = vertex_list[triTable[cubeindex][i  ]][2];
    cloud.push_back (p1);
    p2.x = vertex_list[triTable[cubeindex][i+1]][0];
    p2.y = vertex_list[triTable[cubeindex][i+1]][1];
    p2.z = vertex_list[triTable[cubeindex][i+1]][2];
    cloud.push_back (p2);
    p3.x = vertex_list[triTable[cubeindex][i+2]][0];
    p3.y = vertex_list[triTable[cubeindex][i+2]][1];
    p3.z = vertex_list[triTable[cubeindex][i+2]][2];
    cloud.push_back (p3);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::getNeighborList1D (std::vector<float> &leaf,
                                                Eigen::Vector3i &index3d)
{
  leaf = std::vector<float> (8, 0.0f);

  leaf[0] = getGridValue (index3d);
  leaf[1] = getGridValue (index3d + Eigen::Vector3i (1, 0, 0));
  leaf[2] = getGridValue (index3d + Eigen::Vector3i (1, 0, 1));
  leaf[3] = getGridValue (index3d + Eigen::Vector3i (0, 0, 1));
  leaf[4] = getGridValue (index3d + Eigen::Vector3i (0, 1, 0));
  leaf[5] = getGridValue (index3d + Eigen::Vector3i (1, 1, 0));
  leaf[6] = getGridValue (index3d + Eigen::Vector3i (1, 1, 1));
  leaf[7] = getGridValue (index3d + Eigen::Vector3i (0, 1, 1));
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
  if (!(iso_level_ >= 0 && iso_level_ < 1))
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Invalid iso level %f! Please use a number between 0 and 1.\n", getClassName ().c_str (), iso_level_);
    output.cloud.width = output.cloud.height = 0;
    output.cloud.data.clear ();
    output.polygons.clear ();
    return;
  }

  // Create grid
  grid_ = std::vector<float> (res_x_*res_y_*res_z_, 0.0f);

  // Populate tree
  tree_->setInputCloud (input_);

  getBoundingBox ();

  // Transform the point cloud into a voxel grid
  // This needs to be implemented in a child class
  voxelizeData ();



  // Run the actual marching cubes algorithm, store it into a point cloud,
  // and copy the point cloud + connectivity into output
  pcl::PointCloud<PointNT> cloud;

  for (int x = 1; x < res_x_-1; ++x)
    for (int y = 1; y < res_y_-1; ++y)
      for (int z = 1; z < res_z_-1; ++z)
      {
        Eigen::Vector3i index_3d (x, y, z);
        std::vector<float> leaf_node;
        getNeighborList1D (leaf_node, index_3d);
        createSurface (leaf_node, index_3d, cloud);
      }
  pcl::toPCLPointCloud2 (cloud, output.cloud);

  output.polygons.resize (cloud.size () / 3);
  for (size_t i = 0; i < output.polygons.size (); ++i)
  {
    pcl::Vertices v;
    v.vertices.resize (3);
    for (int j = 0; j < 3; ++j)
      v.vertices[j] = static_cast<int> (i) * 3 + j;
    output.polygons[i] = v;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::performReconstruction (pcl::PointCloud<PointNT> &points,
                                                    std::vector<pcl::Vertices> &polygons)
{
  if (!(iso_level_ >= 0 && iso_level_ < 1))
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Invalid iso level %f! Please use a number between 0 and 1.\n", getClassName ().c_str (), iso_level_);
    points.width = points.height = 0;
    points.points.clear ();
    polygons.clear ();
    return;
  }

  // Create grid
  grid_ = std::vector<float> (res_x_*res_y_*res_z_, 0.0f);

  // Populate tree
  tree_->setInputCloud (input_);

  getBoundingBox ();

  // Transform the point cloud into a voxel grid
  // This needs to be implemented in a child class
  voxelizeData ();

  // Run the actual marching cubes algorithm, store it into a point cloud,
  // and copy the point cloud + connectivity into output
  points.clear ();
  for (int x = 1; x < res_x_-1; ++x)
    for (int y = 1; y < res_y_-1; ++y)
      for (int z = 1; z < res_z_-1; ++z)
      {
        Eigen::Vector3i index_3d (x, y, z);
        std::vector<float> leaf_node;
        getNeighborList1D (leaf_node, index_3d);
        createSurface (leaf_node, index_3d, points);
      }

  polygons.resize (points.size () / 3);
  for (size_t i = 0; i < polygons.size (); ++i)
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

