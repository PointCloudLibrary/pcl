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
 * Author : jpapon@gmail.com
 * Email  : jpapon@gmail.com
 *
 */

#ifndef PCL_SEGMENTATION_VOXEL_SUPERPIXELS_HPP_
#define PCL_SEGMENTATION_VOXEL_SUPERPIXELS_HPP_

#include <pcl/segmentation/voxel_superpixels.h>
#include <pcl/features/fpfh_omp.h>



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::VoxelSuperpixels<PointT>::VoxelSuperpixels (float voxel_resolution, float seed_resolution) :
  resolution_ (voxel_resolution),
  seed_resolution_ (seed_resolution),
  superpixels_ (0),
  point_labels_ (0),
  voxel_cloud_ (),
  seed_cloud_ (),
  superpixel_features_(boost::extents[0][0]),
  seed_indices_ (0),
  voxel_kdtree_ (),
  voxel_octree_ (),
  seed_octree_ (),
  normal_radius_(resolution_*3.0f),
  fpfh_radius_(resolution_*3.0f),
  color_importance_(0.1f),
  spatial_importance_(0.4f),
  fpfh_importance_(0.1f),
  normal_importance_(1.0f),
  normals_ (),
  voxel_fpfh_(),
  seed_indices_orig_ (0),
  seed_indices_unshifted_ (0),
  point_neighbor_dist_ (0),
  voxel_LAB_ (boost::extents[0][0]),
  seed_constituents_ (0),
  voxel_votes_ (0),
  superpixel_colors_ (0)

{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::VoxelSuperpixels<PointT>::~VoxelSuperpixels ()
{

  point_neighbor_dist_.clear ();
  point_labels_.clear ();
  superpixels_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::getNormalRadius () const
{
  return (normal_radius_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setNormalRadius (float radius)
{
  normal_radius_ = radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::getFPFHRadius () const
{
  return (fpfh_radius_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setFPFHRadius (float radius)
{
  fpfh_radius_ = radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::getVoxelResolution () const
{
  return (resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setVoxelResolution (float resolution)
{
  resolution_ = resolution;
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::getSeedResolution () const
{
  return (resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setSeedResolution (float seed_resolution)
{
  seed_resolution_ = seed_resolution;
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::VoxelSuperpixels<PointT>::getVoxelCloudSize () const
{
  if (voxel_cloud_)
    return voxel_cloud_->width;
  else
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
pcl::VoxelSuperpixels<PointT>::getVoxelCloud ()
{
  return voxel_cloud_;
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
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::setNormalImportance (float val)
{
  normal_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<pcl::Normal>::ConstPtr
pcl::VoxelSuperpixels<PointT>::getNormals () const
{
  return (normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::extract (typename pcl::PointCloud<PointT>::Ptr &voxel_cloud, std::vector <pcl::PointIndices>& superpixels)
{
  superpixels_.clear ();
  point_labels_.clear ();
  
  superpixels.clear ();

  std::cout << "Init compute  \n";
  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  std::cout << "Preparing for segmentation \n";
  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  std::cout << "Finding neighbors" << std::endl;
  findPointNeighbors ();
  std::cout << "Placing Seeds" << std::endl;
  placeSeedVoxels ();
  
  std::cout << "Finding seed constituents" << std::endl;
  findSeedConstituency (seed_resolution_*1.5f);
  std::cout << "Evolving the superpixels" << std::endl;
  evolveSuperpixels ();
  //std::cout << "Max cd="<<max_cd_<<"   min cd="<<min_cd_<<std::endl;
  //std::cout << "Max sd="<<max_sd_<<"   min sd="<<min_sd_<<std::endl;
  cleanSuperpixels ();
//  std::cout << "Copying superpixels into output array" << std::endl;
  superpixels.reserve (superpixels_.size ());
  std::copy (superpixels_.begin (), superpixels_.end (), std::back_inserter (superpixels));

  if (!voxel_cloud)
    voxel_cloud = boost::make_shared<pcl::PointCloud<PointT> >();
  pcl::copyPointCloud (*voxel_cloud_, *voxel_cloud);
  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::computeHierarchicalSegmentation ()
{
  //TODO Implement!
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::getSuperpixelNeighbors (std::vector <std::set<int> > &superpixel_neighbors)
{
  std::set<int> neighbors;
  superpixel_neighbors.resize (superpixels_.size (), neighbors);
  // std::cout<< "Finding neighbors, superpixel size="<<superpixel_neighbors.size ()<<"\n";
  for (int i = 0; i < superpixels_.size(); ++i)
  {
    superpixel_neighbors[i].insert (i); //Always touchs itself
    //std::cout << "Finding neighbors for superpixels of "<<i<<"\n";
    for (int k = 0; k < superpixels_[i].indices.size(); ++k)
    {
      int point_index = superpixels_[i].indices[k];
      for (int j = 0; j < point_neighbor_dist_[point_index].size() ; ++j)
      {
        int neighbor_idx = point_neighbor_dist_[point_index][j].first;
        int neighbor_label = label_remapping_[voxel_votes_[neighbor_idx].first];
        if (neighbor_label != -1)
          superpixel_neighbors[i].insert(neighbor_label);
        
      }
    }
    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::getSuperpixelCenters (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > &centers)
{
  pcl::PointXYZ center_point;
  centers.resize (superpixels_.size (), center_point);
  for (int i = 0; i < superpixels_.size(); ++i)
  {
    centers[i].x = superpixel_features_[i][3];
    centers[i].y = superpixel_features_[i][4];
    centers[i].z = superpixel_features_[i][5];
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::getAdjacencyList (std::vector<std::vector<int> > &adjacency_list)
{
  std::vector<int> neighbor_indices;
  adjacency_list.resize (voxel_cloud_->width, neighbor_indices);
  for (int i = 0; i < voxel_cloud_->width; ++i)
  {
    adjacency_list[i].reserve (point_neighbor_dist_[i].size ());
    for (int k = 0; k < point_neighbor_dist_[i].size (); ++k)
    {
      adjacency_list[i].push_back (point_neighbor_dist_[i][k].first);
    }
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
      for (size_t i_segment = 0; i_segment < superpixels_.size ()+1; i_segment++)
      {
        uint8_t r = static_cast<uint8_t>( (rand () % 256));
        uint8_t g = static_cast<uint8_t>( (rand () % 256));
        uint8_t b = static_cast<uint8_t>( (rand () % 256));
        superpixel_colors_.push_back (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      }
    }

    pcl::copyPointCloud (*input_,*colored_cloud);
    
    pcl::PointCloud <pcl::PointXYZRGB>::iterator i_colored;
    typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin ();
    for (i_colored = colored_cloud->begin (); i_colored != colored_cloud->end (); ++i_colored,++i_input)
    {
      int index;
      float sqr_dist;
      i_colored->rgb = 0;     
      if ( !pcl::isFinite<PointT> (*i_input))
        i_colored->rgb = 0;
      else
      {     
        seed_octree_->approxNearestSearch (*i_input, index, sqr_dist);
        //Unlabeled points (-1) just stay black
        if (point_labels_[index] != -1)
          i_colored->rgb = *reinterpret_cast<float*>(&superpixel_colors_ [point_labels_[index]]);
        
      }
      
    }
  }
  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::VoxelSuperpixels<PointT>::getColoredVoxelCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  if (!superpixels_.empty ())
  {
    colored_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> > ();
    if (superpixel_colors_.size () == 0)
    {
      srand (static_cast<unsigned int> (time (0)));
      for (size_t i_segment = 0; i_segment < superpixels_.size ()+1; i_segment++)
      {
        uint8_t r = static_cast<uint8_t>( (rand () % 256));
        uint8_t g = static_cast<uint8_t>( (rand () % 256));
        uint8_t b = static_cast<uint8_t>( (rand () % 256));
        superpixel_colors_.push_back (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        
      }
    }
    pcl::copyPointCloud (*voxel_cloud_,*colored_cloud);
    pcl::PointCloud <pcl::PointXYZRGB>::iterator i_colored;
    typename pcl::PointCloud <PointT>::const_iterator i_input = voxel_cloud_->begin ();
    int index = 0;
    for (i_colored = colored_cloud->begin (); i_colored != colored_cloud->end (); ++i_colored,++i_input)
    {
      i_colored->rgb = 0;
      std::pair <int,float> vote = voxel_votes_[index];
      if (vote.first != -1)
        i_colored->rgb = *reinterpret_cast<float*>(&superpixel_colors_ [vote.first]);
      index++;
    }
  }
  return colored_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZL>::Ptr
pcl::VoxelSuperpixels<PointT>::getLabeledVoxelCloud ()
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud;
  if (!superpixels_.empty ())
  {
    labeled_voxel_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZL> > ();
    
    pcl::copyPointCloud (*voxel_cloud_,*labeled_voxel_cloud);
    //Set all voxel point labels
    for (int i= 0; i < point_labels_.size (); ++i)
    {
      labeled_voxel_cloud->points[i].label = point_labels_[i] + 1;
    }
  }
  
  return labeled_voxel_cloud;  
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
    for (i_labeled = labeled_cloud->begin (); i_labeled != labeled_cloud->end (); ++i_labeled, ++i_input)
    {
      int index;
      float sqr_dist;
      i_labeled->label = 0;
      if ( pcl::isFinite<PointT> (*i_input))
      {  
        seed_octree_->approxNearestSearch (*i_input, index, sqr_dist);
        if (point_labels_[index] != -1)
          i_labeled->label = point_labels_[index] + 1;
      }
    }
  }
  return (labeled_cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::VoxelSuperpixels<PointT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
    return (false);

  
  std::cout << "Computing voxel cloud \n";
  // Voxelize the input cloud
  computeVoxelCloud (input_, voxel_cloud_, input_octree_, resolution_);
  computeVoxelCloud (voxel_cloud_, voxel_cloud_, voxel_octree_, resolution_);
  computeVoxelCloud (voxel_cloud_, seed_cloud_, seed_octree_, seed_resolution_);
  
  std::cout << "Computing Normals \n";
  // Compute normals for voxel cloud
  computeNormals ();
  std::cout << "Computing FPFH \n";
  calcVoxelFPFHValues ();
  
  std::cout << "Computing Lab values \n";
  calcVoxelLABValues();
  
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::findPointNeighbors ()
{
  point_neighbor_dist_.clear ();
  int number_of_points = static_cast<int> (voxel_cloud_->points.size ());
  std::vector<int> k_neighbors;
  std::vector<float> k_sqr_distances;
  std::vector<std::pair <int, float> > actual_neighbor_distances;
  
  int num_neighbors = 26;
  point_neighbor_dist_.resize (voxel_cloud_->points.size (), actual_neighbor_distances);

  k_neighbors.resize (num_neighbors, 0);
  k_sqr_distances.resize (num_neighbors, 0);
  for (int i_point = 0; i_point < number_of_points; i_point++)
  {
    int point_index = (*indices_)[i_point];
    //Searching the voxel cloud here
    voxel_kdtree_->nearestKSearch (i_point, num_neighbors, k_neighbors, k_sqr_distances);
    std::vector<int> actual_neighbors;
    double max_dist_sqr = (resolution_)*(resolution_)*3.0; 
    //Start at one since search returns original point as nearest neighbor
    for (int j = 0; j<k_neighbors.size(); ++j)
    {
      if (k_sqr_distances[j] <= max_dist_sqr)
      {
        actual_neighbors.push_back (k_neighbors[j]);
      //  std::cout << k_sqr_distances[j] <<" < "<<max_dist_sqr<<std::endl;
      }
    }
   // std::cout << "Point "<<i_point<<" has "<<actual_neighbors.size ()<< " neighbors touching it"<<std::endl;
    
    std::pair <int, float> dist_index;
    actual_neighbor_distances.resize (actual_neighbors.size (), dist_index);
    //PointT point = voxel_cloud_->points[point_index];
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
  
  std::vector<PointT, Eigen::aligned_allocator<PointT> > voxel_centers; 
  int num_seeds = seed_octree_->getOccupiedVoxelCenters(voxel_centers); 
  std::cout << "Number of seed points before filtering="<<num_seeds<<std::endl;
  
  
  seed_indices_orig_.resize (num_seeds, 0);
  
  std::vector<int> closest_index;
  std::vector<float> distance;
  closest_index.resize(1,0);
  distance.resize(1,0);
  for (int i = 0; i < num_seeds; ++i)  
  {
    PointT center_point = voxel_centers[i];
    voxel_kdtree_->nearestKSearch (center_point, 1, closest_index, distance);
    //  std::cout << "dist to closest ="<<distance[0]<<std::endl;
    seed_indices_orig_[i] = closest_index[0];
  }
  
  std::vector<int> neighbors;
  std::vector<float> sqr_distances;

  float search_radius = 0.3f*seed_resolution_;
  float merge_radius = 2.0f * resolution_;
  //std::cout << "Search radius = "<<search_radius<<"\n";
  //float search_volume = 4.0f/3.0f * 3.1415926536f * search_radius * search_radius * search_radius;
  // This is number of voxels which fit in a planar slice through search volume
  // Area of planar slice / area of voxel side
  float min_points = 0.5f * (search_radius)*(search_radius) * 3.1415926536f  / (resolution_*resolution_);
  //old = 0.5 * search_volume / (resolution_*resolution_*resolution_);
  std::cout << "Min points search volume = "<<min_points<<std::endl;
  for (int i = 0; i < seed_indices_orig_.size (); ++i)
  {
    int num = voxel_kdtree_->radiusSearch (seed_indices_orig_[i], search_radius , neighbors, sqr_distances);
        //  std::cout << "Found " << neighbors.size () << " points within radius "<<search_radius<<std::endl;
    float min_gradient = calcGradient(seed_indices_orig_[i]);
    int min_index = seed_indices_orig_[i];
    float neighbor_gradient;
    for (int j = 1; j < num; ++j)
    {
      if (sqr_distances[j] < (search_radius*search_radius))
      {
      neighbor_gradient = calcGradient(neighbors[j]);
      if (neighbor_gradient < min_gradient) 
      {
        min_gradient = neighbor_gradient;
        min_index = neighbors[j];
      }
      }
    }
    
    //Check to see if there are any seeds already here, if so, don't add this seed
    voxel_kdtree_->radiusSearch (min_index, merge_radius, neighbors, sqr_distances);
  //  int num_merge = voxel_kdtree_->radiusSearch (seed_indices_orig_[i], merge_radius, neighbors, sqr_distances);
    bool add_seed = true;
    for (int k = 0; k < neighbors.size (); ++k)
    {
      for (int j = 0; j < seed_indices_.size (); ++j)
      {
        if ( seed_indices_[j] == neighbors[k] ) //TODO CHECK color sim, if far, add seed anyways
        {  
          add_seed = false;
          std::cout << "Duplicate, removing seed index "<<i<<"\n";
          break;
        }
      }
      if (!add_seed)
        break;
    }
    
 
   // std::cout<< "num="<<num<<"\n";
    if ( num > min_points && add_seed)
    {
      seed_indices_.push_back (min_index);
      //seed_indices_.push_back (seed_indices_orig_[i]);
      seed_indices_unshifted_.push_back (seed_indices_orig_[i]);
    }
    
    //PointT point = voxel_cloud_->points[min_index];
    // std::cout << point.x<<","<<point.y<<"  idx="<<min_index<<"  ent="<<min_entropy<<std::endl;
    //  superpixels_[i].indices[ii] = neighbors[ii];
    
    //superpixels_[i].indices.swap (neighbors);
  }
  std::cout << "Number of seed points after filtering="<<seed_indices_.size ()<<std::endl;
  pcl::PointIndices superpixel;
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
  if (num_neighbors <= 4)
    return (std::numeric_limits<float>::max ());
  for (int i = 0; i < num_neighbors; ++i)
    gradient += point_neighbor_dist_[point_index][i].second;
  return (gradient / float (num_neighbors));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::findSeedConstituency (float edge_length)
{
 // int number_of_points = static_cast<int> (voxel_cloud_->points.size ());
  size_t number_of_seeds = seed_indices_.size ();

  std::vector<int> constituents;
  std::vector<int> possible_constituents;
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
    int num = seed_octree_->boxSearch (min_pt,max_pt , possible_constituents);
    seed_constituents_[i].reserve (num);
    //Now go through and find which ones are flow connected to this seed
    //Start at seed point, find all of its connected neighbors
    seed_constituents_[i].push_back (seed_index);
    recursiveFind (seed_index, possible_constituents, seed_constituents_[i]);
    //std::cout << "Seed "<<i<<" has "<<seed_constituents_[i].size()<<" constituents\n"; 
    //Sort the constituents based on index (so we can do a binary search on them)
    std::sort (seed_constituents_[i].begin (), seed_constituents_[i].end ());
    
  }
  //
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::VoxelSuperpixels<PointT>::recursiveFind (int index, std::vector<int> &possible_constituents, std::vector<int > &constituents)
{
  std::vector<std::pair<int,float> >::iterator it_neighbors = point_neighbor_dist_[index].begin ();
  
  
  for ( ; it_neighbors != point_neighbor_dist_[index].end (); ++it_neighbors)
  {
    bool already_added = false;
    std::vector< int >::iterator it_constituents = constituents.begin ();
    //Check if this neighbor is already in the list
    for ( ; it_constituents!=constituents.end(); ++it_constituents)
      if ( (*it_constituents) == (*it_neighbors).first)
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
          constituents.push_back ((*it_neighbors).first);
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
 
  point_labels_.resize (voxel_cloud_->points.size (), -1);
  std::pair <int,float> vote_pair (-1, std::numeric_limits<float>::max());;
  voxel_votes_.resize (voxel_cloud_->points.size (),vote_pair);
  //Calculate initial values for clusters
 // std::cout << "Initializing Superpixel Clusters...\n";
  initSuperpixelClusters (resolution_);
  
  int max_depth = static_cast<int>(1.6 * seed_resolution_/resolution_);
  int num_iterations = 1;
  std::cout << "Max depth = "<<max_depth<<"\n";
  
  for (int itr = 0; itr < num_iterations; itr++)
  {
    for (int i = 1; i < max_depth; ++i)
    {
      //Reset all votes!
      for (int k = 0; k < voxel_votes_.size (); ++k)
      {
        voxel_votes_[k].first = -1;
        voxel_votes_[k].second = std::numeric_limits<float>::max();
      }
      //Iterate through, assigning constituents to closest center
      iterateSuperpixelClusters (i);
      
      //Update Superpixel features
      updateSuperpixelClusters ();  
      

    }
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
  //Features are {L a b x y z normal_x normal_y normal_z FPFHx33}
  superpixel_features_.resize (boost::extents[num_seeds][42]);
  std::pair <int, float> def(-1,std::numeric_limits<float>::max());
  voxel_votes_.resize (voxel_cloud_->points.size (),def);
  //std::cout << "Seed Const size="<<seed_constituents_.size ()<<"\n";
  for (int i=0; i<num_seeds; ++i)
  {
    //std::cout <<"Working seed "<<i<<std::endl;
    std::vector <double> temp_sums(42,0);
    std::vector<int >::iterator it_constituents = seed_constituents_[i].begin ();
    int num_in_radius = 0;
    //std::cout <<"Seed has "<<seed_constituents_[i].size ()<<"\n";
    for ( ; it_constituents!=seed_constituents_[i].end() ; ++it_constituents)
    {
      int index = (*it_constituents);
      float dist = sqrtf (calcDistanceSquared (index, seed_indices_[i]));
      if (dist < search_radius)
      {
        num_in_radius++;
        voxel_votes_[index].first = i;
        temp_sums[0] += voxel_LAB_[index][0];
        temp_sums[1] += voxel_LAB_[index][1];
        temp_sums[2] += voxel_LAB_[index][2];
        
        temp_sums[3] += voxel_cloud_->points[index].x;
        temp_sums[4] += voxel_cloud_->points[index].y;
        temp_sums[5] += voxel_cloud_->points[index].z;
        
        if (pcl::isFinite<pcl::Normal>(normals_->points[index]))
        {
          temp_sums[6] += normals_->points[index].normal_x;
          temp_sums[7] += normals_->points[index].normal_y;
          temp_sums[8] += normals_->points[index].normal_z;
        }
        if (pcl::isFinite<pcl::FPFHSignature33>(voxel_fpfh_->points[index]))
        {
          for (int k =9; k<42; ++k)
          {
            temp_sums[k] += voxel_fpfh_->points[index].histogram[k-9];
          }
        }
      }
    }
    for (int k =0; k<42; ++k)
    {
      if (num_in_radius > 0)
        superpixel_features_[i][k] = static_cast<float>(temp_sums[k]/num_in_radius);
     // std::cout<<k<<"     "<<superpixel_features_[i][k]<<std::endl;
    }
    //Move cluster seeds to point nearest center of superpixel
    PointT point;
    point.x = superpixel_features_[i][3];
    point.y = superpixel_features_[i][4];
    point.z = superpixel_features_[i][5];
    std::vector <float> sqr_dist;
    std::vector <int> index;
    //SHIFT THE CENTER?
    voxel_kdtree_->nearestKSearch (point,1, index, sqr_dist);
    seed_indices_[i] = index[0];
    //std::cout<<"--------------------------------------------------\n";
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::iterateSuperpixelClusters (int max_depth)
{
  size_t num_seeds = seed_indices_.size ();
  //Features are {L a b x y z FPFHx33}
  
  std::vector<std::set<int> >checked;
  std::set<int> temp_set;
  checked.resize (num_seeds, temp_set);
  std::vector<std::queue <std::pair<int,float> > >existing_queue;
  std::queue <std::pair<int,float> > temp_queue;
  existing_queue.resize (num_seeds, temp_queue);
  
  //#pragma omp parallel for
  for (int i=0; i<num_seeds; ++i)
  {
    if (seed_indices_[i] == -1)
      continue;
    int debug_count = 0;
  //  std::vector<int>::iterator it_constituents = seed_constituents_[i].begin ();
    //std::vector<int>::iterator it_constituents_end = seed_constituents_[i].end ();
    
    std::vector<std::pair<int,float> >::iterator it_neighbors;
    std::vector<std::pair<int,float> >::iterator it_neighbors_end;
    std::pair<std::set<int>::iterator,bool> checked_ret;
    std::queue <std::pair<int,float> > check_queue;
    //Set seed pixel to label
    float dist = calcFeatureDistance (seed_indices_[i], i);
    if (dist <= voxel_votes_[seed_indices_[i]].second)
    {
      voxel_votes_[seed_indices_[i]].second = dist;  
      voxel_votes_[seed_indices_[i]].first = i;
      checked[i].insert (seed_indices_[i]);
      it_neighbors = point_neighbor_dist_[seed_indices_[i]].begin ();
      it_neighbors_end = point_neighbor_dist_[seed_indices_[i]].end ();
      //Push seed neighbors onto queue - pair = {voxel index, consitutent index}
      for (; it_neighbors!=it_neighbors_end; ++it_neighbors)
      {
        existing_queue[i].push ((*it_neighbors));
        checked[i].insert ((*it_neighbors).first);
        ++debug_count;
      }
    }
    else
    {
      //std::cout << "SEED "<<i<<" IS NOT USING OWN POINT, dist="<<dist<<"  other dist="<< voxel_votes_[seed_indices_[i]].second<<std::endl;
      //seed_indices_[i] = -1;
    }
  }
  
  for (int level = 0; level < max_depth; ++level)
  {
    
   // #pragma omp parallel for
    for (int i=0; i<num_seeds; ++i)
    {
      if (seed_indices_[i] == -1)
        continue;
      int debug_count = 0;
      std::vector<int>::iterator it_constituents = seed_constituents_[i].begin ();
      std::vector<int>::iterator it_constituents_end = seed_constituents_[i].end ();
      
      std::vector<std::pair<int,float> >::iterator it_neighbors;
      std::vector<std::pair<int,float> >::iterator it_neighbors_end;
      std::pair<std::set<int>::iterator,bool> checked_ret;
      std::queue <std::pair<int,float> > check_queue;
           
      //Keep going until we have no more voxels to check 
      //This will be empty if seed doesn't own it's own point
      while (!existing_queue[i].empty ())
      {
        std::pair<int,float> constituent = existing_queue[i].front ();
        existing_queue[i].pop ();
        float dist = calcFeatureDistance (constituent.first, i);
        if (dist < voxel_votes_[constituent.first].second)
        {
          voxel_votes_[constituent.first].second = dist;  
          voxel_votes_[constituent.first].first = i;
          //Find neighbors, if they haven't been checked and are constituents add to check_queue
          it_neighbors = point_neighbor_dist_[constituent.first].begin ();
          it_neighbors_end = point_neighbor_dist_[constituent.first].end ();
          for ( ; it_neighbors != it_neighbors_end; ++it_neighbors)
          {
            //True returned from set::insert means it wasn't in the set before
            if (checked[i].insert((*it_neighbors).first).second == true
              && std::binary_search(it_constituents, it_constituents_end,(*it_neighbors).first))
            {
              check_queue.push (*it_neighbors);
              ++debug_count;
            }
          }
        }
      }
      //Copy check queue into existing_queue for next round
      existing_queue[i] = check_queue;
      //std::cout << "Seed "<<i<<" checked "<<debug_count<<" points\n";
    }
  }
  
  /*
  for (int i=0; i<num_seeds; ++i)
  {
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
 */
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::updateSuperpixelClusters ()
{
  size_t num_seeds = seed_indices_.size ();
  //Features are {L a b x y z normal_x normal_y normal_z FPFHx33}
  
  //std::cout << "Seed Const size="<<seed_constituents_.size ()<<"\n";
  for (int i=0; i<num_seeds; ++i)
  {
    if (seed_indices_[i] == -1)
      continue;
    //std::cout <<"Working seed "<<i<<std::endl;
    std::vector <double> temp_sums(42,0);
    std::vector<int>::iterator it_constituents = seed_constituents_[i].begin ();
    int num_votes = 0;
    //std::cout <<"Seed has "<<seed_constituents_[i].size ()<<"\n";
    for ( ; it_constituents!=seed_constituents_[i].end() ; ++it_constituents)
    {
      int index = (*it_constituents);
      if (voxel_votes_[index].first == i)
      {
        num_votes++;
        temp_sums[0] += voxel_LAB_[index][0];
        temp_sums[1] += voxel_LAB_[index][1];
        temp_sums[2] += voxel_LAB_[index][2];
        temp_sums[3] += voxel_cloud_->points[index].x;
        temp_sums[4] += voxel_cloud_->points[index].y;
        temp_sums[5] += voxel_cloud_->points[index].z;
        
        if (pcl::isFinite<pcl::Normal>(normals_->points[index]))
        {
          temp_sums[6] += normals_->points[index].normal_x;
          temp_sums[7] += normals_->points[index].normal_y;
          temp_sums[8] += normals_->points[index].normal_z;
        }
        if (pcl::isFinite<pcl::FPFHSignature33>(voxel_fpfh_->points[index]))
        {
          for (int k =9; k<42; ++k)
          {
            temp_sums[k] += voxel_fpfh_->points[index].histogram[k-9];
          }
        }
        
      }
    }
    for (int k =0; k<42; ++k)
    {
      //TODO HOW TO HANDLE NUM_VOTES == 0!!!
      if (num_votes > 0)
        superpixel_features_[i][k] = static_cast<float>(temp_sums[k]/num_votes);
      // std::cout<<k<<"     "<<superpixel_features_[i][k]<<std::endl;
    }
    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::cleanSuperpixels ()
{
  
  if (!point_labels_.empty ())
  {
    //Find all seeds which actually have points 
    label_remapping_.clear ();
    label_remapping_.resize (seed_indices_.size(), -1);
    int num_labels = 0;
    for (int i = 0; i < seed_indices_.size (); ++i)
    {
      for(int k = 0; k < point_labels_.size(); ++k)
      {
        if (point_labels_[k] == i)
        {
          label_remapping_[i] = num_labels;
          num_labels++;
          break;
        }
      }
    }
    //Relabel to skip any unused labels
    for (int i = 0; i < point_labels_.size (); ++i)
      if (point_labels_[i] != -1)
        point_labels_[i] = label_remapping_[point_labels_[i]];
     
    pcl::PointIndices superpixel;
    superpixels_.resize(num_labels, superpixel);
  
    //Set the superpixel vector based on new labels
    for (int i = 0; i < point_labels_.size (); ++i)
    {
      if ( point_labels_[i] != -1)
        superpixels_[point_labels_[i]].indices.push_back(i);
    }  
    
    boost::multi_array<float, 2> superpixel_features_temp;
    superpixel_features_temp.resize (boost::extents[superpixels_.size ()][42]);
    for (int i = 0; i < label_remapping_.size (); ++i)
    {
      if (label_remapping_[i] != -1)
        superpixel_features_temp[label_remapping_[i]] = superpixel_features_[i];
    }
    superpixel_features_.resize (boost::extents[superpixels_.size ()][42]);
    superpixel_features_ = superpixel_features_temp;
 }
  
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::computeVoxelCloud (typename pcl::PointCloud<PointT>::ConstPtr input_cloud, 
                                                  typename pcl::PointCloud<PointT>::Ptr& output_cloud,
                                                  typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr &octree, 
                                                  float resolution)
{
  if (output_cloud != 0)
  {
    std::cout << "Voxel cloud not empty, deleting!\n";
    output_cloud.reset ();
  }
  
  if (octree == 0 || (octree->getInputCloud () != input_cloud || octree->getResolution () != resolution))
  {
    octree.reset ();
    std::cout << "Octree empty, not this cloud, or wrong resolution setting and filling it\n";
    octree = boost::make_shared<pcl::octree::OctreePointCloudSearch <PointT> > (resolution);
    octree->setInputCloud (input_cloud);
    octree->addPointsFromInputCloud ();
  }   
  
  output_cloud = boost::make_shared<pcl::PointCloud<PointT> >();
  
  //pcl::VoxelGrid<PointT> vox_grid;
  //vox_grid.setInputCloud (input_cloud);
  //vox_grid.setLeafSize (resolution, resolution, resolution);
  //vox_grid.filter (*output_cloud);
  
  //Use center of voxels, not centroid - faster, and makes touching easier to calculate
  //std::vector<PointT, Eigen::aligned_allocator<PointT> > center_vector;
  octree->getOccupiedVoxelCenters (output_cloud->points);
  int index;
  float sqr_dist;
  for (int i = 0; i < output_cloud->points.size(); ++i)
  {
    octree->approxNearestSearch (output_cloud->points[i],index,sqr_dist);
    output_cloud->points[i].rgba = input_cloud->points[index].rgba;
  }
  //  std::cout <<i<<" color="<< output_cloud->points[i].r<<" "<<output_cloud->points[i].g<<" "<<output_cloud->points[i].b<<std::endl;
  output_cloud->width = static_cast<uint32_t> (output_cloud->points.size ());
  output_cloud->height = 1;
  output_cloud->is_dense = true;
  output_cloud->sensor_origin_ = input_cloud->sensor_origin_;
  output_cloud->sensor_orientation_ = input_cloud->sensor_orientation_;
  std::cout << "Voxel cloud has "<<output_cloud->width<<" occupied voxels\n";
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
  ne.setRadiusSearch (normal_radius_);
  ne.compute (*normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::calcVoxelFPFHValues ()
{
  //Calculate FPFH for all voxels
  if (fpfh_importance_ > 0.0f)
  {
    pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (voxel_cloud_);
    fpfh.setInputNormals (normals_);
    fpfh.setSearchMethod (voxel_kdtree_);
    voxel_fpfh_ = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();
    fpfh.setRadiusSearch (fpfh_radius_);
    fpfh.compute (*voxel_fpfh_);
    for (int i = 0; i < voxel_fpfh_->points.size (); ++i)
    {
      //Normalize
      for (int k = 0; k < 33; ++k)
        voxel_fpfh_->points[i].histogram[k] /= 300.0f;
    }
  }
  else
  {
    voxel_fpfh_ = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();
    pcl::FPFHSignature33 fpfh_point;
    voxel_fpfh_->points.resize (voxel_cloud_->points.size(),fpfh_point);
    for (int i = 0; i < voxel_fpfh_->points.size (); ++i)
    {
      
      for (int k = 0; k < 33; ++k)
        voxel_fpfh_->points[i].histogram[k] = 0.0f;
    }
    
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
  spatial_distance_squared /= (seed_resolution_*seed_resolution_*9);
  
  float fpfh_distance = 1.0f;
  //Histogram intersection kernel
  for(int i = 0; i < 33; ++i)
  {
    if (voxel_fpfh_->points[point_index].histogram[i] < superpixel_features_[seed_index][i+9])
      fpfh_distance -= voxel_fpfh_->points[point_index].histogram[i];
    else
      fpfh_distance -= superpixel_features_[seed_index][i+9];
  }
  
  float similarity = 0.0f;
  if (pcl::isFinite<pcl::Normal>(normals_->points[point_index]))
  {
    similarity += (normals_->points[point_index].normal_x *  superpixel_features_[seed_index][6]) ;
    similarity += (normals_->points[point_index].normal_y * superpixel_features_[seed_index][7]) ;
    similarity += (normals_->points[point_index].normal_z * superpixel_features_[seed_index][8]) ;
  }
   //10000 balances so white to black dist = 1.0
  // Others are already balance to 0 to 1.0 range
  distance += (color_distance_squared)/10000.0f * color_importance_;
  distance += spatial_distance_squared * spatial_importance_;
  distance += fpfh_distance * fpfh_importance_;
  distance += (1.0f - similarity) * (1.0f - similarity)*normal_importance_;
  return ((distance));
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
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::calcNormalScalarProduct (int index_a, int index_b)
{
  float similarity = 0.0f;
  similarity += (normals_->points[index_a].normal_x * normals_->points[index_b].normal_x) ;
  similarity += (normals_->points[index_a].normal_y * normals_->points[index_b].normal_y) ;
  similarity += (normals_->points[index_a].normal_z * normals_->points[index_b].normal_z) ;
  
  return static_cast<float> (fabs (similarity));
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::VoxelSuperpixels<PointT>::getVoxelSeedCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr seed_cloud;
  if (!superpixels_.empty ())
  {
    seed_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> > ();
    srand (static_cast<unsigned int> (time (0)));
    pcl::copyPointCloud (*voxel_cloud_,*seed_cloud);

    uint8_t r = static_cast<uint8_t> (0);
    uint8_t g = static_cast<uint8_t> (255);
    uint8_t b = static_cast<uint8_t> (0);
    uint32_t green = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    r = static_cast<uint8_t> (255); g = static_cast<uint8_t> (0);
    uint32_t red = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    r = static_cast<uint8_t> (0); b = static_cast<uint8_t> (255);
    uint32_t blue = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    std::vector<int> neighbors;
    std::vector<float> sqr_distances;

    typename pcl::search::KdTree<PointT>::Ptr orig_kdtree = boost::make_shared< pcl::search::KdTree<PointT> >();
    orig_kdtree->setInputCloud (input_);
    for (int i = 0; i < seed_indices_orig_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_orig_[i]];
      int num = voxel_kdtree_->radiusSearch (point, 0.25*seed_resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
        seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&red);
      }
    }
    for (int i = 0; i < seed_indices_unshifted_.size (); ++i)
    {
      if(seed_indices_[i] == -1)
        continue;
      PointT point = voxel_cloud_->points[seed_indices_unshifted_[i]];
      int num = voxel_kdtree_->radiusSearch (point, 0.25*seed_resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
        seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&green);
      }
    }
    for (int i = 0; i < seed_indices_.size (); ++i)
    {
      if(seed_indices_[i] == -1)
        continue;
      PointT point = voxel_cloud_->points[seed_indices_[i]];
      int num = voxel_kdtree_->radiusSearch (point, resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
        seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&blue);
      }
    }
  }
  return (seed_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::VoxelSuperpixels<PointT>::getSeedCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr seed_cloud;
  if (!superpixels_.empty ())
  {
    std::cout << "Seed original = " <<seed_indices_orig_.size () <<"\n";
    std::cout << "Seed unsifted = " <<seed_indices_unshifted_.size () <<"\n";
    std::cout << "Seed final = " <<seed_indices_.size () <<"\n";
    
    seed_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> > ();
    srand (static_cast<unsigned int> (time (0)));
    pcl::copyPointCloud (*input_,*seed_cloud);
    
    /*
    uint8_t r = static_cast<uint8_t> (0);
    uint8_t g = static_cast<uint8_t> (255);
    uint8_t b = static_cast<uint8_t> (0);
    uint32_t green = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    r = static_cast<uint8_t> (255); g = static_cast<uint8_t> (0);
    uint32_t red = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    r = static_cast<uint8_t> (0); b = static_cast<uint8_t> (255);
    uint32_t blue = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    */
    
    std::vector<int> neighbors;
    std::vector<float> sqr_distances;
    
    typename pcl::search::KdTree<PointT>::Ptr orig_kdtree = boost::make_shared< pcl::search::KdTree<PointT> >();
    orig_kdtree->setInputCloud (input_);
    for (int i = 0; i < seed_indices_orig_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_orig_[i]];
      int num = orig_kdtree->radiusSearch (point, 0.1*seed_resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
       // seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&red);
      }
    }
    for (int i = 0; i < seed_indices_unshifted_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_unshifted_[i]];
      int num = orig_kdtree->radiusSearch (point, 0.1*seed_resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
      //  seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&green);
      }
    }
    for (int i = 0; i < seed_indices_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_[i]];
      int num = orig_kdtree->radiusSearch (point, resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
        seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&superpixel_colors_ [label_remapping_[i]]);
      }
    }
  }
  return (seed_cloud);
}
#endif    // PCL_SEGMENTATION_VOXEL_SUPERPIXELS_HPP_
 
