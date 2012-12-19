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

#ifndef PCL_OCTREE_POINTCLOUD_SUPERVOXEL_HPP_
#define PCL_OCTREE_POINTCLOUD_SUPERVOXEL_HPP_

#include <pcl/octree/octree_pointcloud_supervoxel.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> 
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::OctreePointCloudSuperVoxel (const double resolution_arg, float max_dist, float color_weight, float normal_weight, float spatial_weight) 
  : OctreePointCloud<PointT, LeafContainerT, BranchContainerT> (resolution_arg)
{
  max_dist_ = max_dist;
  max_dist_sqr_ = max_dist * max_dist;
  color_weight_ = color_weight;
  normal_weight_ = normal_weight;
  spatial_weight_ = spatial_weight;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::computeNeighbors ()
{
  OctreeKey current_key, neighbor_key;
  typename OctreeSuperVoxelT::LeafNodeIterator leaf_itr;
  LeafContainerT *leaf_container, *neighbor_container;
  //Now iterate through finding neighbors, and create edges
  for ( leaf_itr = this->leaf_begin () ; leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    current_key = leaf_itr.getCurrentOctreeKey ();  
    leaf_container = dynamic_cast<LeafContainerT*> (*leaf_itr);
    for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dz = -1; dz <= 1; ++dz)
        {
          neighbor_key.x = current_key.x + dx;
          neighbor_key.y = current_key.y + dy;
          neighbor_key.z = current_key.z + dz;
          LeafNode *neighbor = OctreeBaseT::findLeaf (neighbor_key);
          if (neighbor)
          {
            neighbor_container = dynamic_cast<LeafContainerT*> (neighbor);
            leaf_container->addNeighbor (neighbor_container);
          }
        }
      }
    }
      
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> bool
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::getVoxelCentroidAtPoint (
  const PointT& point_arg, PointSuperVoxel& voxel_centroid_arg) const
{
  OctreeKey key;
  LeafNode* leaf = 0;
  
  // generate key
  genOctreeKeyforPoint (point_arg, key);
  
  leaf = this->findLeaf (key);
  
  if (leaf)
  {
    LeafContainerT* container = leaf;
    container->getCentroid (voxel_centroid_arg);
  }
  
  return (leaf != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> size_t
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::getVoxelCentroids (std::vector<PointSuperVoxel, Eigen::aligned_allocator<PointSuperVoxel> > &voxel_centroids_arg) const
{
  OctreeKey new_key;
  
  // reset output vector
  voxel_centroids_arg.clear ();
  voxel_centroids_arg.reserve (this->leafCount_);
  
  typename OctreeSuperVoxelT::LeafNodeIterator leaf_itr;
  leaf_itr = this->leaf_begin ();
  LeafContainerT* leafContainer;
  int num = 0;
  for ( ; leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    leafContainer = dynamic_cast<LeafContainerT*> (*leaf_itr);
    PointSuperVoxel new_centroid;
    leafContainer->getCentroid (new_centroid);
    voxel_centroids_arg.push_back (new_centroid);
    if (leafContainer->getLabel() != 0)
      ++num;
  }
  //getVoxelCentroidsRecursive (this->rootNode_, new_key, voxel_centroid_list_arg );
 // std::cout << "There are "<<num <<" labeled points!\n";
  // return size of centroid vector
  return (voxel_centroids_arg.size ());
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::insertNormals (const std::vector<PointSuperVoxel, Eigen::aligned_allocator<PointSuperVoxel> > &voxel_centroids_arg)
{
  assert (voxel_centroids_arg.size () == this->leafCount_);
  typename OctreeSuperVoxelT::LeafNodeIterator leaf_itr;
  leaf_itr = this->leaf_begin ();
  LeafContainerT* leafContainer;
  
  std::vector<PointSuperVoxel, Eigen::aligned_allocator<PointSuperVoxel> >::const_iterator it_centroids = voxel_centroids_arg.begin ();
  int num = 0;
  for ( ; leaf_itr != this->leaf_end (); ++leaf_itr, ++it_centroids)
  {
    
    leafContainer = dynamic_cast<LeafContainerT*> (*leaf_itr);
    leafContainer->setCentroid (*it_centroids);
    if (leafContainer->getLabel() != 0)
      ++num;
  }
 // std::cout << "Num labeled in normal insert "<<num<<"\n";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::getVoxelCentroidsRecursive (const BranchNode* branch_arg, OctreeKey& key_arg, std::vector<PointSuperVoxel, Eigen::aligned_allocator<PointSuperVoxel> > &voxel_centroid_list_arg) const
{
  // child iterator
  unsigned char child_idx;
  
  // iterate over all children
  for (child_idx = 0; child_idx < 8; child_idx++)
  {
    // if child exist
    if (branch_arg->hasChild (child_idx))
    {
      // add current branch voxel to key
      key_arg.pushBranch (child_idx);
      
      const OctreeNode *childNode = branch_arg->getChildPtr (child_idx);
      
      switch (childNode->getNodeType ())
      {
        case BRANCH_NODE:
        {
          // recursively proceed with indexed child branch
          getVoxelCentroidsRecursive (static_cast<const BranchNode*> (childNode), key_arg, voxel_centroid_list_arg);
          break;
        }
        case LEAF_NODE:
        {
          const LeafContainerT* container = static_cast<const LeafNode*> (childNode);
          
          PointSuperVoxel new_centroid;
          container->getCentroid (new_centroid);
          
          voxel_centroid_list_arg.push_back (new_centroid);
          break;
        }
        default:
          break;
      }
      
      // pop current branch voxel from key
      key_arg.popBranch ();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::getCentroidCloud (pcl::PointCloud<PointSuperVoxel>::Ptr &cloud )
{
  bool use_existing_points = true;
  if ((cloud == 0) || (cloud->size () != this->OctreeBaseT::getLeafCount ()))
  {
    cloud.reset ();
    cloud = boost::make_shared<pcl::PointCloud<PointSuperVoxel> > ();
    cloud->reserve (this->OctreeBaseT::getLeafCount ());
    use_existing_points = false;
  }
  typename OctreeSuperVoxelT::LeafNodeIterator leaf_itr;
  leaf_itr = this->leaf_begin ();
  
  LeafContainerT* leafContainer;
  PointSuperVoxel point;
  for ( int index = 0; leaf_itr != this->leaf_end (); ++leaf_itr,++index)
  {
    leafContainer = dynamic_cast<LeafContainerT*> (*leaf_itr);
    if (!use_existing_points)
    {
      leafContainer->getCentroid (point);
      cloud->push_back (point);
    }
    else
    {
      leafContainer->getCentroid (cloud->points[index]);
    }
    
  }  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> const pcl::PointSuperVoxel&
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::setPointLabel (const PointSuperVoxel &point_arg, uint32_t label_arg)
{
  OctreeKey key;
  // generate key for point
  PointT point;
  point.x = point_arg.x;
  point.y = point_arg.y;
  point.z = point_arg.z;
  
  this->genOctreeKeyforPoint (point, key);
  // Make sure point exists
  LeafContainerT* leaf = dynamic_cast<LeafContainerT*> (OctreeBaseT::findLeaf (key));
  assert (leaf != 0);
  leaf->setLabel (label_arg);
  leaf->setTempLabel (label_arg);
  leaf->setDistance (0.0f);
  return leaf->getCentroid ();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::computeSuperVoxelAdjacencyGraph (const std::map<uint32_t,PointSuperVoxel> &supervoxel_centers, VoxelAdjacencyList &supervoxel_adjacency_graph)
{
  supervoxel_adjacency_graph.clear ();
  std::map<uint32_t,PointSuperVoxel>::const_iterator it_labels, it_labels_end;
  it_labels_end = supervoxel_centers.end ();
  //Add a vertex for each label, store ids in map
  std::map <uint32_t, VoxelID> label_ID_map;
  for (it_labels = supervoxel_centers.begin (); it_labels!=it_labels_end; ++it_labels)
  {
    VoxelID node_id = add_vertex (supervoxel_adjacency_graph);
    supervoxel_adjacency_graph[node_id] = (it_labels->second);
    label_ID_map.insert (std::make_pair (it_labels->first, node_id));
  }
 
  typename OctreeSuperVoxelT::LeafNodeIterator leaf_itr;
  LeafContainerT *leaf_container;
  //Iterate through and add edges to adjacency graph
  for ( leaf_itr = this->leaf_begin () ; leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    leaf_container = dynamic_cast<LeafContainerT*> (*leaf_itr);
    uint32_t leaf_label = leaf_container->getLabel ();
    if (leaf_label != 0)
    {
      typename std::set<LeafContainerT*>::iterator neighbor_itr;
      typename std::set<LeafContainerT*>::iterator neighbor_end;
      boost::tie (neighbor_itr, neighbor_end) = leaf_container->getAdjacentVoxels ();
      LeafContainerT* neighbor_container;
      for ( ; neighbor_itr != neighbor_end; ++neighbor_itr)
      {
        neighbor_container = *neighbor_itr;
        uint32_t neighbor_label = neighbor_container->getLabel ();
        if ( neighbor_label != 0 && neighbor_label != leaf_label)
        {
          //Labels are different for neighboring voxels, add an edge!
          bool edge_added;
          EdgeID edge;
          VoxelID u = (label_ID_map.find (leaf_label))->second;
          VoxelID v = (label_ID_map.find (neighbor_label))->second;
          boost::tie (edge, edge_added) = add_edge (u,v,supervoxel_adjacency_graph);
          //Calc distance between centers, set as edge weight
          if (edge_added)
          {
            float weight = distance (supervoxel_centers.find (leaf_label)->second
                                     ,supervoxel_centers.find (neighbor_label)->second);
            supervoxel_adjacency_graph[edge].weight = weight;
          }
        }
      }
        
    }
  }

  
  
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::iterateVoxelLabels (const std::map<uint32_t,PointSuperVoxel> &supervoxel_centers)
{
  typename OctreeSuperVoxelT::LeafNodeIterator leaf_itr;
  
  LeafContainerT* leafContainer;

  std::map<uint32_t, PointSuperVoxel>::const_iterator it;
  for ( leaf_itr = this->leaf_begin (); leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    leafContainer = dynamic_cast<LeafContainerT*> (*leaf_itr);
    //If this node is labeled, try to spread its label to all of its neighbors
    if (leafContainer->getLabel () != 0)
    {
      const PointSuperVoxel &point = leafContainer->getCentroid ();
      //Check all neighbors!
      it = supervoxel_centers.find (point.label);
      //Make sure we found the label...
      assert (it != supervoxel_centers.end ());
      checkNeighbors (leafContainer, it->second);
      
    }
  }
  

  for ( leaf_itr = this->leaf_begin (); leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    leafContainer = dynamic_cast<LeafContainerT*> (*leaf_itr);
    leafContainer->pushLabel ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::updateCenters (std::map<uint32_t,PointSuperVoxel> &supervoxel_centers)
{
  typename OctreeSuperVoxelT::LeafNodeIterator leaf_itr;
  leaf_itr = this->leaf_begin ();
  LeafContainerT* leafContainer;
  //Reset the distances to 0, this stores the number of labels
  std::map<uint32_t,PointSuperVoxel>::iterator it = supervoxel_centers.begin ();
  std::map<uint32_t,PointSuperVoxel>::iterator it_end = supervoxel_centers.end ();
  for ( ; it != it_end; ++it)
  {
    it->second.x = it->second.y = it->second.z = 0.0f;
    it->second.R = it->second.G = it->second.B = 0.0f;
    it->second.normal_x = it->second.normal_y = it->second.normal_z = 0.0f;
    it->second.distance = 0.0f;
  }

  for ( ; leaf_itr != this->leaf_end (); ++leaf_itr)
  {
    leafContainer = dynamic_cast<LeafContainerT*> (*leaf_itr);
    if (leafContainer->getLabel () != 0)
    {
      const PointSuperVoxel &point = leafContainer->getCentroid ();
      //Check all neighbors!
      it = supervoxel_centers.find (point.label);
      assert (it != supervoxel_centers.end ());
      //Add in the values, increment counter
      it->second.x += point.x;
      it->second.y += point.y;
      it->second.z += point.z;
      it->second.R += point.R;
      it->second.G += point.G;
      it->second.B += point.B;
      it->second.normal_x += point.normal_x;
      it->second.normal_y += point.normal_y;
      it->second.normal_z += point.normal_z;
      it->second.distance += 1.0f;
    }
  }
  for ( it = supervoxel_centers.begin () ; it != it_end; ++it)
  {
    it->second.x /= it->second.distance;
    it->second.y /= it->second.distance;
    it->second.z /= it->second.distance;
    it->second.R /= it->second.distance;
    it->second.G /= it->second.distance;
    it->second.B /= it->second.distance;
    it->second.normal_x /= it->second.distance;
    it->second.normal_y /= it->second.distance;
    it->second.normal_z /= it->second.distance;
    float norm = std::sqrt (it->second.normal_x * it->second.normal_x
                        + it->second.normal_y * it->second.normal_y
                        + it->second.normal_z * it->second.normal_z);
    it->second.normal_x /= norm;
    it->second.normal_y /= norm;
    it->second.normal_z /= norm;
  }
  
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> void
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::checkNeighbors (const LeafContainerT *leaf_arg, const PointSuperVoxel &supervoxel_center)
{
  float dist;
  typename std::set<LeafContainerT*>::iterator neighbor_itr;
  typename std::set<LeafContainerT*>::iterator neighbor_end;
  
  boost::tie (neighbor_itr, neighbor_end) = leaf_arg->getAdjacentVoxels ();
  LeafContainerT* neighbor_container;
  for ( ; neighbor_itr != neighbor_end; ++neighbor_itr)
  {
    neighbor_container = *neighbor_itr;
    //If the neighbor already has our label, we don't need to check 
    // This isn't strictly true, since the center moves, but if any other label does take it, we will update over it
    //if ( neighbor_container->getLabel () != supervoxel_center.label)
    {
      const PointSuperVoxel &point = neighbor_container->getCentroid ();
      dist = distance (point, supervoxel_center);
      if (dist < point.distance)
      {
        neighbor_container->setTempLabel (supervoxel_center.label);
        neighbor_container->setDistance (dist);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT> float
pcl::octree::OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT>::distance (const PointSuperVoxel &p1, const PointSuperVoxel &p2) const
{

  float spatial_dist = 0.0f, normal_sim = 0.0f, color_dist = 0.0f;
  
  spatial_dist += (p1.x - p2.x)*(p1.x - p2.x);
  spatial_dist += (p1.y - p2.y)*(p1.y - p2.y);
  spatial_dist += (p1.z - p2.z)*(p1.z - p2.z);
  //Short out if we're further than the max dist, we shouldn't be considering this point
  if (spatial_dist > max_dist_sqr_)
    return std::numeric_limits<float>::max();
  spatial_dist /= max_dist_sqr_;
  spatial_dist = std::sqrt(spatial_dist);
  
  
  color_dist += (p1.R - p2.R)*(p1.R - p2.R);
  color_dist += (p1.G - p2.G)*(p1.G - p2.G);
  color_dist += (p1.B - p2.B)*(p1.B - p2.B);
  color_dist = std::sqrt(color_dist)/std::sqrt (3.0f);
  
  normal_sim += p1.normal_x * p2.normal_x;
  normal_sim += p1.normal_y * p2.normal_y;
  normal_sim += p1.normal_z * p2.normal_z;
  normal_sim = normal_sim * normal_sim;
  normal_sim = std::sqrt (normal_sim);
  //std::cout <<"retval="<<(1.0f - normal_sim) * normal_weight_ + color_dist * color_weight_+ spatial_dist * spatial_weight_;
  return  (1.0f - normal_sim) * normal_weight_ + color_dist * color_weight_+ spatial_dist * spatial_weight_;
        
}



#define PCL_INSTANTIATE_OctreePointCloudSuperVoxel(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudSuperVoxel<T>;

#endif

