/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Jeremie Papon
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

#ifndef PCL_OCTREE_POINTCLOUD_ADJACENCY_HPP_
#define PCL_OCTREE_POINTCLOUD_ADJACENCY_HPP_

#include <pcl/console/print.h>
#include <pcl/common/geometry.h>
/*
 * OctreePointCloudAdjacency is not precompiled, since it's used in other
 * parts of PCL with custom LeafContainers. So if PCL_NO_PRECOMPILE is NOT
 * used, octree_pointcloud_adjacency.h includes this file but octree_pointcloud.h
 * would not include the implementation because it's precompiled. So we need to
 * include it here "manually".
 */
#include <pcl/octree/impl/octree_pointcloud.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> 
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::OctreePointCloudAdjacency (const double resolution_arg) 
: OctreePointCloud<PointT, LeafContainerT, BranchContainerT
, OctreeBase<LeafContainerT, BranchContainerT> > (resolution_arg)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::addPointsFromInputCloud ()
{
  //double t1,t2;
  float minX = std::numeric_limits<float>::max (), minY = std::numeric_limits<float>::max (), minZ = std::numeric_limits<float>::max ();
  float maxX = -std::numeric_limits<float>::max(), maxY = -std::numeric_limits<float>::max(), maxZ = -std::numeric_limits<float>::max();
  
  for (size_t i = 0; i < input_->size (); ++i)
  {
    PointT temp (input_->points[i]);
    if (transform_func_) //Search for point with 
      transform_func_ (temp);
    if (!pcl::isFinite (temp)) //Check to make sure transform didn't make point not finite
      continue;
    if (temp.x < minX)
      minX = temp.x;
    if (temp.y < minY)
      minY = temp.y;
    if (temp.z < minZ)
      minZ = temp.z;
    if (temp.x > maxX)
      maxX = temp.x;
    if (temp.y > maxY)
      maxY = temp.y;
    if (temp.z > maxZ)
      maxZ = temp.z;
  }
  this->defineBoundingBox (minX, minY, minZ, maxX, maxY, maxZ);

  OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::addPointsFromInputCloud ();
  
  LeafContainerT *leaf_container;
  typename OctreeAdjacencyT::LeafNodeIterator leaf_itr;
  leaf_vector_.reserve (this->getLeafCount ());
  for ( leaf_itr = this->leaf_begin () ; leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    OctreeKey leaf_key = leaf_itr.getCurrentOctreeKey ();
    leaf_container = &(leaf_itr.getLeafContainer ());
    
    //Run the leaf's compute function
    leaf_container->computeData ();

    computeNeighbors (leaf_key, leaf_container);
    
    leaf_vector_.push_back (leaf_container);
  }
  //Make sure our leaf vector is correctly sized
  assert (leaf_vector_.size () == this->getLeafCount ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::genOctreeKeyforPoint (const PointT& point_arg,OctreeKey & key_arg) const
{
  if (transform_func_)
  {
    PointT temp (point_arg);
    transform_func_ (temp);
   // calculate integer key for transformed point coordinates
    if (pcl::isFinite (temp)) //Make sure transformed point is finite - if it is not, it gets default key
    {
      key_arg.x = static_cast<unsigned int> ((temp.x - this->min_x_) / this->resolution_);
      key_arg.y = static_cast<unsigned int> ((temp.y - this->min_y_) / this->resolution_);
      key_arg.z = static_cast<unsigned int> ((temp.z - this->min_z_) / this->resolution_);
    }
    else
    {
      key_arg = OctreeKey ();
    }
  }
  else 
  {
    // calculate integer key for point coordinates
    key_arg.x = static_cast<unsigned int> ((point_arg.x - this->min_x_) / this->resolution_);
    key_arg.y = static_cast<unsigned int> ((point_arg.y - this->min_y_) / this->resolution_);
    key_arg.z = static_cast<unsigned int> ((point_arg.z - this->min_z_) / this->resolution_);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::addPointIdx (const int pointIdx_arg)
{
  OctreeKey key;
  
  assert (pointIdx_arg < static_cast<int> (this->input_->points.size ()));
  
  const PointT& point = this->input_->points[pointIdx_arg];
  if (!pcl::isFinite (point))
    return;
   
  // generate key
  this->genOctreeKeyforPoint (point, key);
  // add point to octree at key
  LeafContainerT* container = this->createLeaf(key);
  container->addPoint (point);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::computeNeighbors (OctreeKey &key_arg, LeafContainerT* leaf_container)
{ 
  //Make sure requested key is valid
  if (key_arg.x > this->max_key_.x || key_arg.y > this->max_key_.y || key_arg.z > this->max_key_.z)
  {
    PCL_ERROR ("OctreePointCloudAdjacency::computeNeighbors Requested neighbors for invalid octree key\n");
    return;
  }
  
  OctreeKey neighbor_key;
  int dx_min = (key_arg.x > 0) ? -1 : 0;
  int dy_min = (key_arg.y > 0) ? -1 : 0;
  int dz_min = (key_arg.z > 0) ? -1 : 0;
  int dx_max = (key_arg.x == this->max_key_.x) ? 0 : 1;
  int dy_max = (key_arg.y == this->max_key_.y) ? 0 : 1;
  int dz_max = (key_arg.z == this->max_key_.z) ? 0 : 1;
    
  for (int dx = dx_min; dx <= dx_max; ++dx)
  {
    for (int dy = dy_min; dy <= dy_max; ++dy)
    {
      for (int dz = dz_min; dz <= dz_max; ++dz)
      {
        neighbor_key.x = static_cast<uint32_t> (key_arg.x + dx);
        neighbor_key.y = static_cast<uint32_t> (key_arg.y + dy);
        neighbor_key.z = static_cast<uint32_t> (key_arg.z + dz);
        LeafContainerT *neighbor = this->findLeaf (neighbor_key);
        if (neighbor)
        {
          leaf_container->addNeighbor (neighbor);
        }
      }
    }
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> LeafContainerT*
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::getLeafContainerAtPoint (
  const PointT& point_arg) const
{
  OctreeKey key;
  LeafContainerT* leaf = 0;
  // generate key
  this->genOctreeKeyforPoint (point_arg, key);
  
  leaf = this->findLeaf (key);
  
  return leaf;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::computeVoxelAdjacencyGraph (VoxelAdjacencyList &voxel_adjacency_graph)
{
  //TODO Change this to use leaf centers, not centroids!
  
  voxel_adjacency_graph.clear ();
  //Add a vertex for each voxel, store ids in map
  std::map <LeafContainerT*, VoxelID> leaf_vertex_id_map;
  for (typename OctreeAdjacencyT::LeafNodeIterator leaf_itr = this->leaf_begin () ; leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    OctreeKey leaf_key = leaf_itr.getCurrentOctreeKey ();
    PointT centroid_point;
    this->genLeafNodeCenterFromOctreeKey (leaf_key, centroid_point);
    VoxelID node_id = add_vertex (voxel_adjacency_graph);
    
    voxel_adjacency_graph[node_id] = centroid_point;
    LeafContainerT* leaf_container = &(leaf_itr.getLeafContainer ());
    leaf_vertex_id_map[leaf_container] = node_id;
  }
  
  //Iterate through and add edges to adjacency graph
  for ( typename std::vector<LeafContainerT*>::iterator leaf_itr = leaf_vector_.begin (); leaf_itr != leaf_vector_.end (); ++leaf_itr)
  {
    typename LeafContainerT::iterator neighbor_itr = (*leaf_itr)->begin ();
    typename LeafContainerT::iterator neighbor_end = (*leaf_itr)->end ();
    LeafContainerT* neighbor_container;
    VoxelID u = (leaf_vertex_id_map.find (*leaf_itr))->second;
    PointT p_u = voxel_adjacency_graph[u];
    for ( ; neighbor_itr != neighbor_end; ++neighbor_itr)
    {
      neighbor_container = *neighbor_itr;
      EdgeID edge;
      bool edge_added;
      VoxelID v = (leaf_vertex_id_map.find (neighbor_container))->second;
      boost::tie (edge, edge_added) = add_edge (u,v,voxel_adjacency_graph);
      
      PointT p_v = voxel_adjacency_graph[v];
      float dist = (p_v.getVector3fMap () - p_u.getVector3fMap ()).norm ();
      voxel_adjacency_graph[edge] = dist;
      
    }
      
  }
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> bool
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::testForOcclusion (const PointT& point_arg, const PointXYZ &camera_pos)
{
  OctreeKey key;
  this->genOctreeKeyforPoint (point_arg, key);
  // This code follows the method in Octree::PointCloud
  Eigen::Vector3f sensor(camera_pos.x,
                         camera_pos.y,
                         camera_pos.z);
  
  Eigen::Vector3f leaf_centroid(static_cast<float> ((static_cast<double> (key.x) + 0.5f) * this->resolution_ + this->min_x_),
                                static_cast<float> ((static_cast<double> (key.y) + 0.5f) * this->resolution_ + this->min_y_), 
                                static_cast<float> ((static_cast<double> (key.z) + 0.5f) * this->resolution_ + this->min_z_));
  Eigen::Vector3f direction = sensor - leaf_centroid;
  
  float norm = direction.norm ();
  direction.normalize ();
  float precision = 1.0f;
  const float step_size = static_cast<const float> (resolution_) * precision;
  const int nsteps = std::max (1, static_cast<int> (norm / step_size));
  
  OctreeKey prev_key = key;
  // Walk along the line segment with small steps.
  Eigen::Vector3f p = leaf_centroid;
  PointT octree_p;
  for (int i = 0; i < nsteps; ++i)
  {
    //Start at the leaf voxel, and move back towards sensor.
    p += (direction * step_size);
    
    octree_p.x = p.x ();
    octree_p.y = p.y ();
    octree_p.z = p.z ();
    //  std::cout << octree_p<< "\n";
    OctreeKey key;
    this->genOctreeKeyforPoint (octree_p, key);
    
    // Not a new key, still the same voxel (starts at self).
    if ((key == prev_key))
      continue;
    
    prev_key = key;
    
    LeafContainerT *leaf = this->findLeaf (key);
    //If the voxel is occupied, there is a possible occlusion
    if (leaf)
    {
     return true;
    }
  }
  
  //If we didn't run into a voxel on the way to this camera, it can't be occluded.
  return false;
  
}

#define PCL_INSTANTIATE_OctreePointCloudAdjacency(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudAdjacency<T>;

#endif

