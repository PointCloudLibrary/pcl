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
  : data_ (), leaves_ (), min_p_ (), max_p_ (), leaf_size_ (), gaussian_scale_ (), 
    data_size_ (), padding_size_ (), iso_level_ (), cell_hash_map_ ()
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

  Eigen::Vector4f bounding_box_size = max_p_ - min_p_;

  bounding_box_size = max_p_ - min_p_;
  PCL_DEBUG ("[pcl::MarchingCubes::getBoundingBox] Size of Bounding Box is [%f, %f, %f]\n",
      bounding_box_size.x (), bounding_box_size.y (), bounding_box_size.z ());
  double max_size =
    (std::max) ((std::max)(bounding_box_size.x (), bounding_box_size.y ()),
                bounding_box_size.z ());

  data_size_ = static_cast<uint64_t> (max_size / leaf_size_);
  PCL_DEBUG ("[pcl::MarchingCubes::getBoundingBox] Lower left point is [%f, %f, %f]\n",
      min_p_.x (), min_p_.y (), min_p_.z ());
  PCL_DEBUG ("[pcl::MarchingCubes::getBoundingBox] Upper left point is [%f, %f, %f]\n",
      max_p_.x (), max_p_.y (), max_p_.z ());
  PCL_DEBUG ("[pcl::MarchingCubes::getBoundingBox] Padding size: %d\n", padding_size_);
  PCL_DEBUG ("[pcl::MarchingCubes::getBoundingBox] Leaf size: %f\n", leaf_size_);

  gaussian_scale_ = pow ((padding_size_+1) * leaf_size_ / 2.0, 2.0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::interpolateEdge (float iso_level,
                                              Eigen::Vector3f &p1,
                                              Eigen::Vector3f &p2,
                                              float val_p1,
                                              float val_p2,
                                              Eigen::Vector3f &output)
{
  if (iso_level - val_p1 < 0.00001)
    output = p1;
  if (iso_level - val_p2 < 0.00001)
    output = p2;
  if (val_p1 - val_p2 < 0.00001)
    output = p1;

  float mu = (iso_level - val_p1) / (val_p2-val_p1);
  output[0] = p1[0] + mu * (p2[0] - p1[0]);
  output[1] = p1[1] + mu * (p2[1] - p1[1]);
  output[2] = p1[2] + mu * (p2[2] - p1[2]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::createSurface (Leaf leaf_node,
                                            Eigen::Vector3i &index_3d,
                                            pcl::PointCloud<PointNT> &cloud,
                                            float iso_level)
{
  int cubeindex = 0;
  Eigen::Vector3f vertex_list[12];
  if (leaf_node.vertex[0] < iso_level) cubeindex |= 1;
  if (leaf_node.vertex[1] < iso_level) cubeindex |= 2;
  if (leaf_node.vertex[2] < iso_level) cubeindex |= 4;
  if (leaf_node.vertex[3] < iso_level) cubeindex |= 8;
  if (leaf_node.vertex[4] < iso_level) cubeindex |= 16;
  if (leaf_node.vertex[5] < iso_level) cubeindex |= 32;
  if (leaf_node.vertex[6] < iso_level) cubeindex |= 64;
  if (leaf_node.vertex[7] < iso_level) cubeindex |= 128;

  // Cube is entirely in/out of the surface
  if (edgeTable[cubeindex] == 0)
     return;

  std::vector<Eigen::Vector3f> p;
  p.resize (8);
  Eigen::Vector4f center;
  getCellCenterFromIndex (index_3d, center);
  for (int i = 0; i < 8; ++i)
  {
    Eigen::Vector3f point;
    if(i & 0x4)
      point[1] = static_cast<float> (center[1] + leaf_size_/2);
    else
      point[1] = static_cast<float> (center[1] - leaf_size_/2);

    if(i & 0x2)
      point[2] = static_cast<float> (center[2] + leaf_size_/2);
    else
      point[2] = static_cast<float> (center[2] - leaf_size_/2);

    if((i & 0x1) ^ ((i >> 1) & 0x1))
      point[0] = static_cast<float> (center[0] + leaf_size_/2);
    else
      point[0] = static_cast<float> (center[0] - leaf_size_/2);

    p[i] = point;
  }

  // Find the vertices where the surface intersects the cube
  if (edgeTable[cubeindex] & 1)
    interpolateEdge (iso_level,p[0],p[1],leaf_node.vertex[0],leaf_node.vertex[1], vertex_list[0]);
  if (edgeTable[cubeindex] & 2)
    interpolateEdge (iso_level,p[1],p[2],leaf_node.vertex[1],leaf_node.vertex[2], vertex_list[1]);
  if (edgeTable[cubeindex] & 4)
    interpolateEdge (iso_level,p[2],p[3],leaf_node.vertex[2],leaf_node.vertex[3], vertex_list[2]);
  if (edgeTable[cubeindex] & 8)
    interpolateEdge (iso_level,p[3],p[0],leaf_node.vertex[3],leaf_node.vertex[0], vertex_list[3]);
  if (edgeTable[cubeindex] & 16)
    interpolateEdge (iso_level,p[4],p[5],leaf_node.vertex[4],leaf_node.vertex[5], vertex_list[4]);
  if (edgeTable[cubeindex] & 32)
    interpolateEdge (iso_level,p[5],p[6],leaf_node.vertex[5],leaf_node.vertex[6], vertex_list[5]);
  if (edgeTable[cubeindex] & 64)
    interpolateEdge (iso_level,p[6],p[7],leaf_node.vertex[6],leaf_node.vertex[7], vertex_list[6]);
  if (edgeTable[cubeindex] & 128)
    interpolateEdge (iso_level,p[7],p[4],leaf_node.vertex[7],leaf_node.vertex[4], vertex_list[7]);
  if (edgeTable[cubeindex] & 256)
    interpolateEdge (iso_level,p[0],p[4],leaf_node.vertex[0],leaf_node.vertex[4], vertex_list[8]);
  if (edgeTable[cubeindex] & 512)
    interpolateEdge (iso_level,p[1],p[5],leaf_node.vertex[1],leaf_node.vertex[5], vertex_list[9]);
  if (edgeTable[cubeindex] & 1024)
    interpolateEdge (iso_level,p[2],p[6],leaf_node.vertex[2],leaf_node.vertex[6], vertex_list[10]);
  if (edgeTable[cubeindex] & 2048)
    interpolateEdge (iso_level,p[3],p[7],leaf_node.vertex[3],leaf_node.vertex[7], vertex_list[11]);

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
pcl::MarchingCubes<PointNT>::getNeighborList1D (Leaf leaf,
                                                Eigen::Vector3i &index3d,
                                                HashMap &neighbor_list)
{
  for (int x = -1; x < 2; ++x)
  {
    for (int y = -1; y < 2; ++y)
    {
      for (int z = -1; z < 2; ++z)
      {
        if (x == 0 && y == 0 && z == 0)
          continue;

        if (index3d[0] == 0 && x == -1)
          continue;
        if (index3d[1] == 0 && y == -1)
          continue;
        if (index3d[2] == 0 && z == -1)
          continue;

        Eigen::Vector3i neighbor_index;
        neighbor_index[0] = index3d[0] + x;
        neighbor_index[1] = index3d[1] + y;
        neighbor_index[2] = index3d[2] + z;
        Leaf neighbor_leaf;

        if ((x == 0 || x == 1) &&
            (y == 0 || y == 1) &&
            (z == 0 || z == 1))
          neighbor_leaf.vertex[0] = leaf.vertex[0];

        if ((x == 0 || x == -1) &&
            (y == 0 || y == 1) &&
            (z == 0 || z == 1))
          neighbor_leaf.vertex[1] = leaf.vertex[1];

        if ((x == 0 || x == -1) &&
            (y == 0 || y == 1) &&
            (z == 0 || z == -1))
          neighbor_leaf.vertex[2] = leaf.vertex[2];

        if ((x == 0 || x == 1) &&
            (y == 0 || y == 1) &&
            (z == 0 || z == -1))
          neighbor_leaf.vertex[3] = leaf.vertex[3];

        if ((x == 0 || x == 1) &&
            (y == 0 || y == -1) &&
            (z == 0 || z == 1))
          neighbor_leaf.vertex[4] = leaf.vertex[4];

        if ((x == 0 || x == -1) &&
            (y == 0 || y == -1) &&
            (z == 0 || z == 1))
          neighbor_leaf.vertex[5] = leaf.vertex[5];

        if ((x == 0 || x == -1) &&
            (y == 0 || y == -1) &&
            (z == 0 || z == -1))
          neighbor_leaf.vertex[6] = leaf.vertex[6];

        if ((x == 0 || x == 1) &&
            (y == 0 || y == -1) &&
            (z == 0 || z == -1))
          neighbor_leaf.vertex[7] = leaf.vertex[7];

        neighbor_list[getIndexIn1D (neighbor_index)] = neighbor_leaf;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> void
pcl::MarchingCubes<PointNT>::performReconstruction (pcl::PolygonMesh &output)
{
  if (!(iso_level_ > 0 && iso_level_ < 1))
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Invalid iso level %f! Please use a number between 0 and 1.\n", getClassName ().c_str (), iso_level_);
    output.cloud.width = output.cloud.height = 0;
    output.cloud.data.clear ();
    output.polygons.clear ();
    return;
  }
  if (leaf_size_ <=0)
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Invalid leaf size %f! Please use a number greater than 0.\n", getClassName ().c_str (), leaf_size_);
    output.cloud.width = output.cloud.height = 0;
    output.cloud.data.clear ();
    output.polygons.clear ();
    return;
    
  }

  getBoundingBox ();

  // transform the point cloud into a voxel grid
  // this needs to be implemented in a child class
  voxelizeData ();

  // run the actual marching cubes algorithm, store it into a point cloud,
  // and copy the point cloud + connectivity into output
  pcl::PointCloud<PointNT> cloud;
  BOOST_FOREACH (typename HashMap::value_type entry, cell_hash_map_)
  {
    Eigen::Vector3i leaf_index;
    getIndexIn3D (entry.first, leaf_index);
    createSurface (entry.second, leaf_index, cloud, iso_level_);
  }
  pcl::toROSMsg (cloud, output.cloud);
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
  /// TODO these should go to initCompute ()
  if (!(iso_level_ > 0 && iso_level_ < 1))
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Invalid iso level %f! Please use a number between 0 and 1.\n", getClassName ().c_str (), iso_level_);
    points.width = points.height = 0;
    points.clear ();
    polygons.clear ();
    return;
  }
  if (leaf_size_ <=0)
  {
    PCL_ERROR ("[pcl::%s::performReconstruction] Invalid leaf size %f! Please use a number greater than 0.\n", getClassName ().c_str (), leaf_size_);
    points.width = points.height = 0;
    points.clear ();
    polygons.clear ();
    return;

  }

  getBoundingBox ();

  // transform the point cloud into a voxel grid
  // this needs to be implemented in a child class
  voxelizeData ();

  // run the actual marching cubes algorithm, store it into a point cloud,
  // and copy the point cloud + connectivity into output
  BOOST_FOREACH (typename HashMap::value_type entry, cell_hash_map_)
  {
    Eigen::Vector3i leaf_index;
    getIndexIn3D (entry.first, leaf_index);
    createSurface (entry.second, leaf_index, points, iso_level_);
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

