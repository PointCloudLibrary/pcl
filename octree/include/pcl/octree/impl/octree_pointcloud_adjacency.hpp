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

#include <pcl/octree/octree_pointcloud_adjacency.h>

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
  
  //t1 = timer_.getTime ();
  OctreePointCloud<PointT, LeafContainerT, BranchContainerT>::addPointsFromInputCloud ();


  //t2 = timer_.getTime ();
  //std::cout << "Add Points:"<<t2-t1<<" ms  Num leaves ="<<this->getLeafCount ()<<"\n";
   
  std::list <std::pair<OctreeKey,LeafContainerT*> > delete_list;
  //double t_temp, t_neigh, t_compute, t_getLeaf;
  //t_neigh = t_compute = t_getLeaf = 0;
  LeafContainerT *leaf_container;
  typename OctreeAdjacencyT::LeafNodeIterator leaf_itr;
  leaf_vector_.reserve (this->getLeafCount ());
  for ( leaf_itr = this->leaf_begin () ; leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    //t_temp = timer_.getTime ();
    OctreeKey leaf_key = leaf_itr.getCurrentOctreeKey ();
    leaf_container = &(leaf_itr.getLeafContainer ());
    //t_getLeaf += timer_.getTime () - t_temp;
    
    //t_temp = timer_.getTime ();
    //Run the leaf's compute function
    leaf_container->computeData ();
    //t_compute += timer_.getTime () - t_temp;
     
    //t_temp = timer_.getTime ();
    //  std::cout << "Computing neighbors\n";
    computeNeighbors (leaf_key, leaf_container);
    //t_neigh += timer_.getTime () - t_temp;
    
    leaf_vector_.push_back (leaf_container);

  }
  //Go through and delete voxels scheduled
  for (typename std::list<std::pair<OctreeKey,LeafContainerT*> >::iterator delete_itr = delete_list.begin (); delete_itr != delete_list.end (); ++delete_itr)
  {
    leaf_container = delete_itr->second;
    //Remove pointer to it from all neighbors
    typename std::set<LeafContainerT*>::iterator neighbor_itr = leaf_container->begin ();
    typename std::set<LeafContainerT*>::iterator neighbor_end = leaf_container->end ();
    for ( ; neighbor_itr != neighbor_end; ++neighbor_itr)
    {
      //Don't delete self neighbor
      if (*neighbor_itr != leaf_container)
        (*neighbor_itr)->removeNeighbor (leaf_container);
    }
    this->removeLeaf (delete_itr->first);
  }
  
  //Make sure our leaf vector is correctly sized
  assert (leaf_vector_.size () == this->getLeafCount ());
  
 //  std::cout << "Time spent getting leaves ="<<t_getLeaf<<"\n";
 // std::cout << "Time spent computing data in leaves="<<t_compute<<"\n";
 // std::cout << "Time spent computing neighbors="<<t_neigh<<"\n";
  
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
    key_arg.x = static_cast<unsigned int> ((temp.x - this->min_x_) / this->resolution_);
    key_arg.y = static_cast<unsigned int> ((temp.y - this->min_y_) / this->resolution_);
    key_arg.z = static_cast<unsigned int> ((temp.z - this->min_z_) / this->resolution_);
    
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
  
  if (transform_func_)
  { 
    PointT temp (point);
    transform_func_ (temp);
    this->adoptBoundingBoxToPoint (temp);
  }
  else  
    this->adoptBoundingBoxToPoint (point);
    
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
  
  OctreeKey neighbor_key;
  
  for (int dx = -1; dx <= 1; ++dx)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dz = -1; dz <= 1; ++dz)
      {
        neighbor_key.x = key_arg.x + dx;
        neighbor_key.y = key_arg.y + dy;
        neighbor_key.z = key_arg.z + dz;
        LeafContainerT *neighbor = this->findLeaf (neighbor_key);
        if (neighbor)
        {
          leaf_container->addNeighbor (neighbor);
        }
      }
    }
  }
    
  /* KDTREE Version - probably faster, but cheating...
  PointT &point = leaf_container->getCentroid ();
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  OctreeKey neighbor_key;
  LeafContainerT *neighbor_container;
  //Maximally has 26 neighbors
  int num_neighbors = tree_->radiusSearch (point, max_dist_, k_indices, k_sqr_distances, 26);
  for (int i = 0; i < num_neighbors; ++i)
  {
    std::cout << k_sqr_distances[i] << " ";
    this->genOctreeKeyforPoint (centroid_cloud_->points[i], neighbor_key);
    LeafNode *neighbor = this->findLeafCached (neighbor_key);
    neighbor_container = dynamic_cast<LeafContainerT*> (neighbor);
    //Reflexive add, since neighbor won't always be new as well
    leaf_container->addNeighbor (neighbor_container);
    neighbor_container->addNeighbor (leaf_container);
  }
  std::cout << "\n";
  */
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> LeafContainerT*
pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT, BranchContainerT>::getLeafContainerAtPoint (
  const PointT& point_arg) const
{
  OctreeKey key;
  LeafContainerT* leaf = 0;
  // generate key
  genOctreeKeyforPoint (point_arg, key);
  
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
    genLeafNodeCenterFromOctreeKey (leaf_key, centroid_point);
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
  genOctreeKeyforPoint (point_arg, key);
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

