/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Jeremie Papon
 * Email  : jpapon@gmail.com
 *
 */

#ifndef PCL_SEGMENTATION_VOXEL_SUPERPIXELS_HPP_
#define PCL_SEGMENTATION_VOXEL_SUPERPIXELS_HPP_

#include <pcl/segmentation/voxel_superpixels.h>
#include <pcl/features/fpfh_omp.h>
#include <queue>
#include <list>
#include <cmath>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::VoxelSuperpixels<PointT>::VoxelSuperpixels () :
  min_pts_per_superpixel_ (1),
  max_pts_per_superpixel_ (std::numeric_limits<int>::max ()),
  resolution_ (0.01),
  seed_resolution_ (0.1),
  search_ (),
  color_importance_(1.0),
  spatial_importance_(1.0),
  fpfh_importance_(1.0),
  normals_ (),
  labeled_voxel_cloud_ (),
  voxel_cloud_ (),
  voxel_kdtree_ (),
  voxel_fpfh_(),
  seed_indices_ (0),
  seed_indices_orig_ (0),
  seed_indices_shifted_ (0),
  point_neighbors_ (0),
  point_neighbor_dist_ (0),
  point_labels_ (0),
  num_pts_in_superpixel_ (0),
  superpixels_ (0),
  voxel_LAB_ (boost::extents[0][0]),
  seed_constituents_ (0),
  voxel_votes_ (0),
  superpixel_features_(boost::extents[0][0]),
  number_of_superpixels_ (0),
  superpixel_colors_ (0)
{
  max_cd_=max_sd_=max_fd_= 0.0f;
  min_cd_=min_sd_=min_fd_= 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::VoxelSuperpixels<PointT>::~VoxelSuperpixels ()
{
  if (search_ != 0)
    search_.reset ();
  if (normals_ != 0)
    normals_.reset ();

  point_neighbors_.clear ();
  point_neighbor_dist_.clear ();
  point_labels_.clear ();
  num_pts_in_superpixel_.clear ();
  superpixels_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::VoxelSuperpixels<PointT>::getMinSuperpixelSize () const
{
  return (min_pts_per_superpixel_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setMinSuperpixelSize (int min_superpixel_size)
{
  min_pts_per_superpixel_ = min_superpixel_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::VoxelSuperpixels<PointT>::getMaxSuperpixelSize () const
{
  return (max_pts_per_superpixel_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setMaxSuperpixelSize (int max_superpixel_size)
{
  max_pts_per_superpixel_ = max_superpixel_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::VoxelSuperpixels<PointT>::getVoxelResolution () const
{
  return (resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setVoxelResolution (double resolution)
{
  resolution_ = resolution;
  // if search set
  if (search_ != 0)
    search_->setResolution (resolution);
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> double
pcl::VoxelSuperpixels<PointT>::getSeedResolution () const
{
  return (resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setSeedResolution (double seed_resolution)
{
  seed_resolution_ = seed_resolution;
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setColorImportance (float val)
{
  color_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setSpatialImportance (float val)
{
  spatial_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setFPFHImportance (float val)
{
  fpfh_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<pcl::Normal>::ConstPtr
pcl::VoxelSuperpixels<PointT>::getNormals () const
{
  return (normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::getSeedIndices (std::vector<int>& seed_indices)
{
  seed_indices.reserve (seed_indices_.size ());
  std::copy (seed_indices_.begin (), seed_indices_.end (), std::back_inserter (seed_indices));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::extract (std::vector<pcl::PointIndices>& superpixels)
{
  superpixels_.clear ();
  superpixels.clear ();
  point_neighbors_.clear ();
  point_neighbor_dist_.clear ();
  point_labels_.clear ();
  num_pts_in_superpixel_.clear ();
  number_of_superpixels_ = 0;

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  findPointNeighbors ();
  placeSeedVoxels ();

  findSeedConstituency (float (seed_resolution_) * 1.5f);
  evolveSuperpixels ();
  
  superpixels_ = superpixels;

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::VoxelSuperpixels<PointT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
    return (false);

  // Voxelize the input cloud
  computeVoxelCloud ();
  // Compute normals for voxel cloud
  computeNormals ();
  calcVoxelFPFHValues ();
  
  calcVoxelLABValues();
  
  // if search not set
  if (search_ == 0)
    search_ = boost::make_shared<pcl::octree::OctreePointCloudSearch <PointT> > (resolution_);

  search_->setInputCloud (voxel_cloud_);
  
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::findPointNeighbors ()
{
  int number_of_points = static_cast<int> (voxel_cloud_->points.size ());
  std::vector<int> k_neighbors;
  std::vector<float> k_sqr_distances;
  std::vector<std::pair <int, float> > actual_neighbor_distances;
  
  int num_neighbors = 9;
  point_neighbor_dist_.resize (voxel_cloud_->points.size (), actual_neighbor_distances);

  k_neighbors.resize (num_neighbors, 0);
  k_sqr_distances.resize (num_neighbors, 0);
  for (int i_point = 0; i_point < number_of_points; i_point++)
  {
    int point_index = (*indices_)[i_point];
    //Searching the voxel cloud here
    voxel_kdtree_->nearestKSearch (i_point, num_neighbors, k_neighbors, k_sqr_distances);
    std::vector<int> actual_neighbors;
    double max_dist_sqr = (resolution_)*(resolution_)*3.0; //sqrt(3) = 1.732
    //Start at one since search returns original point as nearest neighbor
    for (int j = 0; j<k_neighbors.size(); ++j)
    {
      if (k_sqr_distances[j] < max_dist_sqr)
      {
        actual_neighbors.push_back (k_neighbors[j]);
      }
    }
    
    std::pair <float, int> dist_index;
    actual_neighbor_distances.resize (actual_neighbors.size (), dist_index);
    for (int i_neighbor = 0; i_neighbor < actual_neighbors.size(); ++i_neighbor)
    {
      int neighbor_index = actual_neighbors[i_neighbor];
      //PointT neighbor_point = voxel_cloud_->points[neighbor_index];
      actual_neighbor_distances[i_neighbor].first = neighbor_index;
      actual_neighbor_distances[i_neighbor].second = calcColorDifferenceLAB(neighbor_index,point_index) ;
    }
    point_neighbor_dist_[point_index].swap (actual_neighbor_distances);
    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::placeSeedVoxels ()
{
  search_->setResolution (seed_resolution_);
  search_->addPointsFromInputCloud ();
  
  std::vector<PointT, Eigen::aligned_allocator<PointT> > voxel_centers; 
  int num_seeds = search_->getOccupiedVoxelCenters(voxel_centers); 
  
  
  seed_indices_orig_.resize (num_seeds, 0);
  
  std::vector<int> closest_index;
  std::vector<float> distance;
  closest_index.resize(1,0);
  distance.resize(1,0);
  for (int i = 0; i < num_seeds; ++i)  
  {
    //PointT center_point = voxel_centers[i];
//    int num = voxel_kdtree_->nearestKSearch (center_point, 1, closest_index, distance);
    seed_indices_orig_[i] = closest_index[0];
  }
  
  //search_->deleteTree ();
  //search_.reset ();
  //search_ = boost::make_shared<pcl::octree::OctreePointCloudSearch <PointT> > (resolution_);
  //search_->setInputCloud (voxel_cloud_);
  //search_->setResolution (resolution_);
  //search_->addPointsFromInputCloud ();
  //search_->addPointsFromInputCloud ();
  std::vector<int> neighbors;
  std::vector<float> sqr_distances;

  pcl::PointIndices superpixel;
  
  float search_radius = 0.1f * float (seed_resolution_);
//  float search_volume = 4.0/3.0 * 3.1415926536 * search_radius * search_radius * search_radius;
  // This is number of voxels which fit in a planar slice through search volume
  // Area of planar slice / area of voxel side
  float min_points = 0.45f * search_radius * search_radius * 3.1415926536f / (float (resolution_ * resolution_));
  //old = 0.5 * search_volume / (resolution_*resolution_*resolution_);
  for (int i = 0; i < seed_indices_orig_.size (); ++i)
  {
    int num = voxel_kdtree_->radiusSearch (seed_indices_orig_[i], search_radius , neighbors, sqr_distances);
    float min_gradient = calcGradient(seed_indices_orig_[i]);
    int min_index = seed_indices_orig_[i];
    float neighbor_gradient;
    for (int j = 1; j < num; ++j)
    {
      neighbor_gradient = calcGradient(neighbors[j]);
      if (neighbor_gradient < min_gradient) 
      {
        min_gradient = neighbor_gradient;
        min_index = neighbors[j];
      }
      
    }
    if ( num > min_points)
    {
      //seed_indices_.push_back (seed_indices_orig_[i]);
      seed_indices_.push_back (min_index);
    }
    
    //PointT point = voxel_cloud_->points[min_index];
    //  superpixels_[i].indices[ii] = neighbors[ii];
    
    //superpixels_[i].indices.swap (neighbors);
  }
  superpixels_.resize(seed_indices_.size (), superpixel);
  for (int i = 0; i < seed_indices_.size (); ++i)
    superpixels_[i].indices.push_back (seed_indices_[i]);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::calcGradient (int point_index)
{
  float gradient = 0.0f; 
  size_t num_neighbors = point_neighbor_dist_[point_index].size ();
  if (num_neighbors <= 4 )
    return std::numeric_limits<float>::max();
  for (int i = 0; i < num_neighbors; ++i)
  {
    gradient += point_neighbor_dist_[point_index][i].second;
  }
  return (gradient / float (num_neighbors));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::findSeedConstituency (float edge_length)
{
//  int number_of_points = static_cast<int> (voxel_cloud_->points.size ());
  size_t number_of_seeds = seed_indices_.size ();

  std::vector< std::pair<int,float> > constituents;
  std::vector< int > possible_constituents;
  seed_constituents_.resize (number_of_seeds, constituents);
  
  
  for ( int i = 0; i < number_of_seeds; ++i)
  {
    int seed_index = seed_indices_[i];
    PointT seed_point = voxel_cloud_->points[seed_index];
    Eigen::Vector3f min_pt(seed_point.x - edge_length/2.0f,
                           seed_point.y - edge_length/2.0f,
                           seed_point.z - edge_length/2.0f);
    Eigen::Vector3f max_pt(seed_point.x + edge_length/2.0f,
                           seed_point.y + edge_length/2.0f,
                           seed_point.z + edge_length/2.0f);
    
    //Find all voxels within the search volume
    int num = search_->boxSearch (min_pt,max_pt , possible_constituents);
    seed_constituents_[i].reserve (num);
    //Now go through and find which ones are flow connected to this seed
    //Start at seed point, find all of its connected neighbors
    std::pair<int,float> seed_index_dist(seed_index, 0.0f);
    seed_constituents_[i].push_back (seed_index_dist);
    recursiveFind (seed_index, possible_constituents, seed_constituents_[i]);
  }
  
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::VoxelSuperpixels<PointT>::recursiveFind (int index, std::vector<int> &possible_constituents, std::vector<std::pair<int,float> > &constituents)
{
  std::vector<std::pair<int,float> >::iterator it_neighbors = point_neighbor_dist_[index].begin ();
  
  
  for ( ; it_neighbors != point_neighbor_dist_[index].end (); ++it_neighbors)
  {
    bool already_added = false;
    std::vector<std::pair<int,float> >::iterator it_constituents = constituents.begin ();
    //Check if this neighbor is already in the list
    for ( ; it_constituents!=constituents.end(); ++it_constituents)
      if ( (*it_constituents).first == (*it_neighbors).first)
      {
        already_added = true;
        break;
      }
    if (already_added)
      continue;
    else   
    { 
      //Check if this point is a possible constituent
      std::vector<int>::iterator it_possible_constituents = possible_constituents.begin ();
      for ( ; it_possible_constituents!=possible_constituents.end(); ++it_possible_constituents)
        if ( *it_possible_constituents == (*it_neighbors).first)
        {
          //Add this index to constituents
          //constituent[0] is seed point
          std::pair<int,float> index_dist_pair ((*it_neighbors).first, calcDistanceSquared((*it_neighbors).first,constituents[0].first));
          constituents.push_back (index_dist_pair);
          //Do a recursive call on the neighbor
          recursiveFind ((*it_neighbors).first,possible_constituents,constituents);
          break;
        }
        
    }
    
  }
}

  



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::evolveSuperpixels ()
{
  size_t num_superpixels = superpixels_.size ();
  
  point_labels_.resize (voxel_cloud_->points.size (), -1);
  num_pts_in_superpixel_.resize (num_superpixels, 0);
  
  //Calculate initial values for clusters
  initSuperpixelClusters (float (resolution_) * 2.0f);
  
  int num_itr = 5;
  for (int i = 0; i < num_itr; ++i)
  {
    //Iterate through, assigning constituents to closest center
    iterateSuperpixelClusters ();
    //Update Superpixel features
    updateSuperpixelClusters ();
  }
  
  //Set the label vector based on votes
  for (int i = 0; i < voxel_votes_.size (); ++i)
  {
    point_labels_[i] = voxel_votes_[i].first;
    
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::initSuperpixelClusters (float search_radius)
{
  size_t num_seeds = seed_indices_.size ();
  //Features are {L a b x y z FPFHx33}
  superpixel_features_.resize (boost::extents[num_seeds][39]);
  std::pair <int, float> def(-1,std::numeric_limits<float>::max());
  voxel_votes_.resize (voxel_cloud_->points.size (),def);
  for (int i=0; i<num_seeds; ++i)
  {
    std::vector <double> temp_sums(39,0);
    std::vector<std::pair<int,float> >::iterator it_constituents = seed_constituents_[i].begin ();
    int num_in_radius = 0;
    for ( ; it_constituents!=seed_constituents_[i].end() ; ++it_constituents)
    {
      int index = (*it_constituents).first;
      float dist = (*it_constituents).second;
      if ( dist < search_radius)
      {
        num_in_radius++;
        voxel_votes_[index].first = i;
        temp_sums[0] += voxel_LAB_[index][0];
        temp_sums[1] += voxel_LAB_[index][1];
        temp_sums[2] += voxel_LAB_[index][2];
        temp_sums[3] += voxel_cloud_->points[index].x;
        temp_sums[4] += voxel_cloud_->points[index].y;
        temp_sums[5] += voxel_cloud_->points[index].z;
        if (pcl::isFinite<pcl::FPFHSignature33>(voxel_fpfh_->points[index]))
        {
          for (int k =6; k<39; ++k)
          {
            temp_sums[k] += voxel_fpfh_->points[index].histogram[k-6];
          }
        }
      }
    }
    for (int k =0; k<39; ++k)
    {
      superpixel_features_[i][k] = static_cast<float>(temp_sums[k]/num_in_radius);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::iterateSuperpixelClusters ()
{
  size_t num_seeds = seed_indices_.size ();
  //Features are {L a b x y z FPFHx33}
  superpixel_features_.resize (boost::extents[num_seeds][39]);
  
  for (int i=0; i<num_seeds; ++i)
  {
    std::vector <double> temp_sums(39,0);
    std::vector<std::pair<int,float> >::iterator it_constituents = seed_constituents_[i].begin ();
    for ( ; it_constituents != seed_constituents_[i].end(); ++it_constituents)
    {
      float dist = calcFeatureDistance ((*it_constituents).first, i);
      if (dist < voxel_votes_[(*it_constituents).first].second)
      {
        voxel_votes_[(*it_constituents).first].second = dist;  
        voxel_votes_[(*it_constituents).first].first = i;
      }
    }
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::updateSuperpixelClusters ()
{
  size_t num_seeds = seed_indices_.size ();
  //Features are {L a b x y z FPFHx33}
  
  for (int i=0; i<num_seeds; ++i)
  {
    std::vector <double> temp_sums(39,0);
    std::vector<std::pair<int,float> >::iterator it_constituents = seed_constituents_[i].begin ();
    int num_votes = 0;
    for ( ; it_constituents!=seed_constituents_[i].end() ; ++it_constituents)
    {
      int index = (*it_constituents).first;
      if (voxel_votes_[index].first == i)
      {
        num_votes++;
        temp_sums[0] += voxel_LAB_[index][0];
        temp_sums[1] += voxel_LAB_[index][1];
        temp_sums[2] += voxel_LAB_[index][2];
        temp_sums[3] += voxel_cloud_->points[index].x;
        temp_sums[4] += voxel_cloud_->points[index].y;
        temp_sums[5] += voxel_cloud_->points[index].z;
        if (pcl::isFinite<pcl::FPFHSignature33>(voxel_fpfh_->points[index]))
        {
          for (int k =6; k<39; ++k)
          {
            temp_sums[k] += voxel_fpfh_->points[index].histogram[k-6];
          }
        }
      }
    }
    for (int k =0; k<39; ++k)
    {
      superpixel_features_[i][k] = static_cast<float>(temp_sums[k]/num_votes);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::cleanSuperpixels ()
{
 
  // TODO
  
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::computeLabeledVoxelCloud ()
{
  if (!superpixels_.empty ())
  {
    labeled_voxel_cloud_ = boost::make_shared< pcl::PointCloud<pcl::PointXYZL> > ();
    
    pcl::copyPointCloud (*voxel_cloud_,*labeled_voxel_cloud_);

    /*std::vector< pcl::PointIndices >::iterator i_segment;
    for (i_segment = superpixels_.begin (); i_segment != superpixels_.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
      {
        int index;
        index = *i_point;
        labeled_voxel_cloud_->points[index].l = i_segment.index;
      }
    }*/
    //Set all voxel point labels
    for (int i= 0; i < point_labels_.size (); ++i)
    {
      labeled_voxel_cloud_->points[i].label = point_labels_[i] + 1;
    }
  }
  
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::VoxelSuperpixels<PointT>::getColoredCloud ()
{
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  if (!superpixels_.empty ())
  {
    colored_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> > ();

    if (superpixel_colors_.size () == 0)
    {
      srand (static_cast<unsigned int> (time (0)));
      //label 0 is unlabeled, should be black
      superpixel_colors_.push_back (0);
      for (size_t i_segment = 0; i_segment < superpixels_.size (); i_segment++)
      {
        uint8_t r = static_cast<unsigned char> (rand () % 200 + 50);
        uint8_t g = static_cast<unsigned char> (rand () % 256);
        uint8_t b = static_cast<unsigned char> (rand () % 256);
        superpixel_colors_.push_back (uint32_t (r) << 16 | uint32_t (g) << 8 | uint32_t (b));
      }
    }
    pcl::copyPointCloud (*input_,*colored_cloud);

    pcl::PointCloud <pcl::PointXYZRGB>::iterator i_colored;
    typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin ();
//    int num_points_not_in_voxel = 0;
    for (i_colored = colored_cloud->begin (); i_colored != colored_cloud->end (); ++i_colored,++i_input)
    {
      int index;
      float sqr_dist;
      i_colored->rgb = 0;
      if ( !pcl::isFinite<PointT> (*i_input))
        i_colored->rgb = 0;
      else
      {
        search_->approxNearestSearch (*i_input, index, sqr_dist);
      //  if (sqr_dist > resolution_ * resolution_*3.0f)
      //  {
        //  num_points_not_in_voxel++;
          //i_colored->rgb = 0;
      //  }
       // else
      //  {
          if (point_labels_[index] != -1)
            i_colored->rgb = *reinterpret_cast<float*>(&superpixel_colors_ [point_labels_[index] + 1]);
       // }
      }

    }
  }
  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZL>::Ptr
pcl::VoxelSuperpixels<PointT>::getLabeledCloud ()
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud;
  if (!superpixels_.empty ())
  {
    labeled_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZL> > ();

    pcl::copyPointCloud (*input_,*labeled_cloud);

    pcl::PointCloud <pcl::PointXYZL>::iterator i_labeled;
    typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin ();
//    int next_color = 0;
    for (i_labeled = labeled_cloud->begin (); i_labeled != labeled_cloud->end (); ++i_labeled, ++i_input)
    {
      int index;
      float sqr_dist;
      i_labeled->label = 0;
      if ( pcl::isFinite<PointT> (*i_input))
      {  
        search_->approxNearestSearch (*i_input, index, sqr_dist);
        if (point_labels_[index] != -1)
          i_labeled->label = point_labels_[index] + 1;
          
      }
      

    }
    
  }

  return (labeled_cloud);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::computeVoxelCloud ()
{
  if (voxel_cloud_ != 0)
    voxel_cloud_.reset ();
  
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setInputCloud (input_);
  vox_grid.setLeafSize (float (resolution_), float (resolution_), float (resolution_));

  voxel_cloud_ = boost::make_shared<pcl::PointCloud<PointT> >();
  vox_grid.filter (*voxel_cloud_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::computeNormals ()
{
  if (normals_ != 0)
    normals_.reset ();
  
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  ne.setInputCloud (voxel_cloud_);
  
  if (voxel_kdtree_ != 0)
    voxel_kdtree_.reset ();
  
  voxel_kdtree_ = boost::make_shared< pcl::search::KdTree<PointT> >();
  ne.setSearchMethod (voxel_kdtree_);
  
  normals_ = boost::make_shared< pcl::PointCloud<pcl::Normal> >();
  
  ne.setRadiusSearch (resolution_*1.8f);
  ne.compute (*normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::calcVoxelFPFHValues ()
{
  //Calculate FPFH for all voxels
  pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (voxel_cloud_);
  fpfh.setInputNormals (normals_);
  fpfh.setSearchMethod (voxel_kdtree_);
  voxel_fpfh_ = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();
  fpfh.setRadiusSearch (resolution_*2.0f);
  fpfh.compute (*voxel_fpfh_);
  for (int i = 0; i < voxel_fpfh_->points.size (); ++i)
  {
    //Normalize
    for (int k = 0; k < 33; ++k)
      voxel_fpfh_->points[i].histogram[k] /= 300.0f;
  }
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::calcVoxelLABValues ()
{
  voxel_LAB_.resize (boost::extents [voxel_cloud_->size ()][3]);
  float r,g,b;
  float X,Y,Z;
  
  for (int i = 0 ; i < voxel_cloud_->size (); ++i)
  {
    r = (voxel_cloud_->points[i]).r / 255.0f;
    g = (voxel_cloud_->points[i]).g / 255.0f;
    b = (voxel_cloud_->points[i]).b / 255.0f;
    if (r > 0.04045f)
      r = std::pow ( (r+0.055f)/1.055f, 2.4f );
    else
      r /= 12.92f;
    
    if (g > 0.04045f)
      g = std::pow ( (g+0.055f)/1.055f, 2.4f );
    else
      g /= 12.92f;
    
    if (b > 0.04045f)
      b = std::pow ( (b+0.055f)/1.055f, 2.4f );
    else
      b /= 12.92f;
    
    r *= 100.0f; g *= 100.0f; b *= 100.0f;
    
    X = r * 0.4124f + g * 0.3576f + b * 0.1805f;
    Y = r * 0.2126f + g * 0.7152f + b * 0.0722f;
    Z = r * 0.0193f + g * 0.1192f + b * 0.9505f;
    
    X /= 95.047f;
    Y /= 100.00f;
    Z /= 108.883f;
    if ( X > 0.008856f) 
      X = std::pow (X, 1.0f/3.0f);
    else
      X = (7.787f * X) + (16.0f / 116.0f);
    
    if ( Y > 0.008856f) 
      Y = std::pow (Y, 1.0f/3.0f);
    else
      Y = (7.787f * Y) + (16.0f / 116.0f);
    
    if ( Z > 0.008856f) 
      Z = std::pow (Z, 1.0f/3.0f);
    else
      Z = (7.787f * Z) + (16.0f / 116.0f);
    
    voxel_LAB_[i][0] = (116.0f * Y) - 16.0f;
    voxel_LAB_[i][1] = 500.0f * ( X - Y );
    voxel_LAB_[i][2] = 200.0f * ( Y - Z );
  }
  
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::calcFeatureDistance (int point_index, int seed_index)
{
  //Features are {L a b x y z FPFHx33}
  float distance = 0.0f;
  float color_distance_squared = 0.0f;
  color_distance_squared += float ((voxel_LAB_[point_index][0] - superpixel_features_[seed_index][0])*(voxel_LAB_[point_index][0] - superpixel_features_[seed_index][0]));
  color_distance_squared += float ((voxel_LAB_[point_index][1] - superpixel_features_[seed_index][1])*(voxel_LAB_[point_index][1] - superpixel_features_[seed_index][1]));
  color_distance_squared += float ((voxel_LAB_[point_index][2] - superpixel_features_[seed_index][2])*(voxel_LAB_[point_index][2] - superpixel_features_[seed_index][2]));
  
  float spatial_distance_squared = 0.0f;
  spatial_distance_squared += (voxel_cloud_->points[point_index].x - superpixel_features_[seed_index][3]) * (voxel_cloud_->points[point_index].x - superpixel_features_[seed_index][3]);
  spatial_distance_squared += (voxel_cloud_->points[point_index].y - superpixel_features_[seed_index][4]) * (voxel_cloud_->points[point_index].y - superpixel_features_[seed_index][4]);
  spatial_distance_squared += (voxel_cloud_->points[point_index].z - superpixel_features_[seed_index][5]) * (voxel_cloud_->points[point_index].z - superpixel_features_[seed_index][5]);
  spatial_distance_squared /= (float (seed_resolution_*seed_resolution_) * 9.0f);
  
  float fpfh_distance = 1.0f;
  //Histogram intersection kernel
  for(int i = 0; i < 33; ++i)
  {
    if (voxel_fpfh_->points[point_index].histogram[i] < superpixel_features_[seed_index][i+6])
      fpfh_distance -= voxel_fpfh_->points[point_index].histogram[i];
    else
      fpfh_distance -= superpixel_features_[seed_index][i+6];
  }
  

  if ( color_distance_squared > max_cd_)
    max_cd_ = color_distance_squared;
  if ( spatial_distance_squared > max_sd_)
    max_sd_ = spatial_distance_squared;

  //10000 balances so white to black dist = 1.0
  // Others are already balance to 0 to 1.0 range
  distance += (color_distance_squared)/10000.0f * color_importance_*color_importance_;
  distance += spatial_distance_squared * spatial_importance_*spatial_importance_;
  distance += fpfh_distance * fpfh_importance_*fpfh_importance_;
  return (sqrtf (distance));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::calcColorDifference (const PointT &a, const PointT &b)
{
  float difference = 0.0f;
  difference += float ((a.r - b.r)*(a.r - b.r));
  difference += float ((a.g - b.g)*(a.g - b.g));
  difference += float ((a.b - b.b)*(a.b - b.b));
  return (difference);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::calcColorDifferenceLAB (int index_a, int index_b)
{
  float difference = 0.0f;
  difference += float ((voxel_LAB_[index_a][0] - voxel_LAB_[index_b][0])*(voxel_LAB_[index_a][0] - voxel_LAB_[index_b][0]));
  difference += float ((voxel_LAB_[index_a][1] - voxel_LAB_[index_b][1])*(voxel_LAB_[index_a][1] - voxel_LAB_[index_b][1]));
  difference += float ((voxel_LAB_[index_a][2] - voxel_LAB_[index_b][2])*(voxel_LAB_[index_a][2] - voxel_LAB_[index_b][2]));
  return (sqrtf (difference));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::calcDifferenceCurvature (int index_a, int index_b)
{
  float difference = 0.0f;
  difference = (normals_->points[index_a].curvature - normals_->points[index_b].curvature);
  return (difference * difference);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::calcDistanceSquared (int index_a, int index_b)
{
  float distance = 0.0f;
  distance += (voxel_cloud_->points[index_a].x - voxel_cloud_->points[index_b].x) * (voxel_cloud_->points[index_a].x - voxel_cloud_->points[index_b].x);
  distance += (voxel_cloud_->points[index_a].y - voxel_cloud_->points[index_b].y) * (voxel_cloud_->points[index_a].y - voxel_cloud_->points[index_b].y);
  distance += (voxel_cloud_->points[index_a].z - voxel_cloud_->points[index_b].z) * (voxel_cloud_->points[index_a].z - voxel_cloud_->points[index_b].z);
  
  return (distance);
  
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::VoxelSuperpixels<PointT>::getSeedCloud ()
{
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr seed_cloud;
  if (!superpixels_.empty ())
  {
    seed_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> > ();
    
    srand (static_cast<unsigned int> (time (0)));

    
    pcl::copyPointCloud (*input_,*seed_cloud);

    uint8_t r = 0;
    uint8_t g = 255;
    uint8_t b = 0;
    uint32_t green = (static_cast<uint32_t> (r) << 16) | (static_cast<uint32_t> (g) << 8) | static_cast<uint32_t> (b);
    r = 255; g = 0;
    uint32_t red = (static_cast<uint32_t> (r) << 16) | (static_cast<uint32_t> (g) << 8) | static_cast<uint32_t> (b);
    r = 0; b = 255;
    uint32_t blue = (static_cast<uint32_t> (r) << 16) | (static_cast<uint32_t> (g) << 8) | static_cast<uint32_t> (b);
    std::vector<int> neighbors;
    std::vector<float> sqr_distances;

    typename pcl::search::KdTree<PointT>::Ptr orig_kdtree = boost::make_shared< pcl::search::KdTree<PointT> >();
    orig_kdtree->setInputCloud (input_);

      
    for (int i = 0; i < seed_indices_orig_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_orig_[i]];
      int num = orig_kdtree->radiusSearch (point, 0.5*seed_resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
        seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&red);
      }
    }
    
    for (int i = 0; i < seed_indices_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_[i]];
      int num = orig_kdtree->radiusSearch (point, 0.5*seed_resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
        seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&green);
      }
    }
    for (int i = 0; i < seed_indices_shifted_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_shifted_[i]];
      int num = orig_kdtree->radiusSearch (point, resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
        seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&blue);
      }
    }
  
  }
  return (seed_cloud);
}
#endif    // PCL_SEGMENTATION_VOXEL_SUPERPIXELS_HPP_
 
