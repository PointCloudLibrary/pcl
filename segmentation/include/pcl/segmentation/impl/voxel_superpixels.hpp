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
  normals_ (),
  labeled_voxel_cloud_ (),
  voxel_cloud_ (),
  voxel_kdtree_ (),
  seed_indices_ (0),
  seed_indices_orig_ (0),
  seed_indices_shifted_ (0),
  point_neighbors_ (0),
  point_neighbor_dist_ (0),
  evolving_set_ (0),
  point_labels_ (0),
  num_pts_in_superpixel_ (0),
  superpixels_ (0),
  voxel_LAB_ (boost::extents[0][0]),
  number_of_superpixels_ (0),
  superpixel_colors_ (0)
{
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
pcl::VoxelSuperpixels<PointT>::extract (std::vector <pcl::PointIndices>& superpixels)
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

  std::cout << "Placing Seeds" << std::endl;
  placeSeedVoxels ();
  std::cout << "Calculating LAB Values"<<std::endl;
  calcVoxelLABValues ();
  std::cout << "Finding neighbors" << std::endl;
  findPointNeighbors ();
  std::cout << "Evolving the superpixels" << std::endl;
  evolveSuperpixels ();

  std::cout << "Copying superpixels into output array" << std::endl;
  superpixels.reserve (superpixels_.size ());
  std::copy (superpixels_.begin (), superpixels_.end (), std::back_inserter (superpixels));

  //deinitCompute ();
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
  std::vector<std::pair <float, int> > actual_neighbor_distances;
  
  
  int num_neighbors = 9;
  point_dist_neighbor_.resize (voxel_cloud_->points.size (), actual_neighbor_distances);
  //TODO: Remove these
 // point_neighbors_.resize (input_->points.size (), k_neighbors);
 // point_neighbor_dist_.resize (input_->points.size (), k_distances);
  
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
      //  std::cout << k_sqr_distances[j] <<" < "<<max_dist_sqr<<std::endl;
      }
    }
   // std::cout << "Point "<<i_point<<" has "<<actual_neighbors.size ()<< " neighbors touching it"<<std::endl;
    
    std::pair <float, int> dist_index;
    actual_neighbor_distances.resize (actual_neighbors.size (), dist_index);
    PointT point = voxel_cloud_->points[point_index];
    for (int i_neighbor = 0; i_neighbor < actual_neighbors.size(); ++i_neighbor)
    {
      int neighbor_index = actual_neighbors[i_neighbor];
      PointT neighbor_point = voxel_cloud_->points[neighbor_index];
      //actual_neighbor_distances[i_neighbor].first = sqrt(calcColorDifference(point,neighbor_point));
      actual_neighbor_distances[i_neighbor].first = sqrt(calcColorDifferenceLAB(neighbor_index,point_index)); 
                                                    + 10.0f*sqrt(calcDifferenceCurvature(neighbor_index,point_index));
     // std::cout << "dist = "<<actual_neighbor_distances[i_neighbor].first<<std::endl;
      actual_neighbor_distances[i_neighbor].second = neighbor_index;
    }
    point_dist_neighbor_[point_index].swap (actual_neighbor_distances);
    
    //TODO: THESE AREN'T NEEDED
  //  point_neighbors_[point_index].swap (actual_neighbors);
  // point_neighbors_dist_[point_index].swap (actual_neighbor_distances);
    
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
  std::cout << "Number of seed points before filtering="<<num_seeds<<std::endl;
  
  
  seed_indices_orig_.resize (num_seeds, 0);
  
  std::vector<int> closest_index;
  std::vector<float> distance;
  closest_index.resize(1,0);
  distance.resize(1,0);
  for (int i = 0; i < num_seeds; ++i)  
  {
    PointT center_point = voxel_centers[i];
    int num = search_->nearestKSearch (center_point, 1, closest_index, distance);
  //  std::cout << "dist to closest ="<<distance[0]<<std::endl;
    seed_indices_orig_[i] = closest_index[0];
  }
  
  //search_->deleteTree ();
 // std::cout << "Setting resolu";
  search_.reset ();
  search_ = boost::make_shared<pcl::octree::OctreePointCloudSearch <PointT> > (resolution_);
  search_->setInputCloud (voxel_cloud_);
  search_->setResolution (resolution_);
  search_->addPointsFromInputCloud ();
 // std::cout << "Done...";
  //search_->addPointsFromInputCloud ();
  //Find points in radius 1/4 of seed resolution around seed points
  std::vector<int> neighbors;
  std::vector<float> sqr_distances;
  std::vector<int> k_neighbors;
  std::vector<float> k_distances;
  int num_neighbors = 8;
  k_neighbors.resize(num_neighbors,0);
  k_distances.resize(num_neighbors,0);
  std::vector<float> entropies;
  pcl::PointIndices superpixel;
 
  float search_radius = 0.4*seed_resolution_;
  float search_volume = 4.0/3.0 * 3.1415926536 * search_radius * search_radius * search_radius;
  // This is number of voxels which fit in a planar slice through search volume
  // Area of planar slice / area of voxel side
  float min_points = 0.45 * (search_radius)*(search_radius) * 3.1415926536  / (resolution_*resolution_);
  //old = 0.5 * search_volume / (resolution_*resolution_*resolution_);
  std::cout << "Min points search volume = "<<min_points<<std::endl;
  for (int i = 0; i < seed_indices_orig_.size (); ++i)
  {
    int num = search_->radiusSearch (seed_indices_orig_[i], search_radius , neighbors, sqr_distances);
  //  std::cout << "Found " << neighbors.size () << " points within radius "<<search_radius<<std::endl;
    float min_curvature = normals_->points[seed_indices_orig_[i]].curvature;
    int min_index = seed_indices_orig_[i];
    //Welford's Method
    float mean = normals_->points[neighbors[0]].curvature;;
    float std_dev = 0.0;
    for (int j = 1; j < num; ++j)
    {
      float curvature = normals_->points[neighbors[j]].curvature;
      float old_mean = mean;
      mean += (curvature - old_mean) / (j);
      std_dev += (curvature - old_mean) * (curvature - mean);
      if (curvature < min_curvature) 
      {
        min_curvature = curvature;
        min_index = neighbors[j];
      }
      
    }
    std_dev = std::sqrt(std_dev/(num -1));
    //std::cout << "Std Dev = "<<std_dev<< "   Mean = "<<mean<<" num pts="<<num<<std::endl;
    if ( std_dev < 0.04 && num > min_points)
    {
      seed_indices_.push_back (seed_indices_orig_[i]);
      seed_indices_shifted_.push_back (min_index);
    }
    //PointT point = voxel_cloud_->points[min_index];
   // std::cout << point.x<<","<<point.y<<"  idx="<<min_index<<"  ent="<<min_entropy<<std::endl;
    //  superpixels_[i].indices[ii] = neighbors[ii];
    
    //superpixels_[i].indices.swap (neighbors);
  }
  std::cout << "Number of seed points after filtering="<<seed_indices_.size ()<<std::endl;
  superpixels_.resize(seed_indices_.size (), superpixel);
  for (int i = 0; i < seed_indices_.size (); ++i)
    superpixels_[i].indices.push_back (seed_indices_[i]);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::evolveSuperpixels ()
{
  int num_superpixels = superpixels_.size ();
  
  point_labels_.resize (voxel_cloud_->points.size (), -1);
  num_pts_in_superpixel_.resize (num_superpixels, 0);
  
  std::list<std::vector<std::pair<float, int> > > label_evolving_set;
  std::vector<std::pair<float, int> > active_pixel;
  std::pair<float, int> dist_to_index;
  
  evolving_set_.resize (num_superpixels, label_evolving_set);
  //Initialize each superpixel using its seed
  for (int i = 0; i<num_superpixels; ++i)
  {
    int seed_idx = seed_indices_[i];
    active_pixel = point_dist_neighbor_[seed_idx];
    evolving_set_[i].push_back (active_pixel);
    point_labels_[seed_idx] = i;
    num_pts_in_superpixel_[i]++;
  }
  
  
  int num_steps = 300;
  float flow_r = 0.4;
  for (int i = 0; i < num_steps; ++i)
  {
   // std::cout << "Iterating..."<<std::endl;
    iterateEvolvingSet (flow_r);  
  }
  
  
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::VoxelSuperpixels<PointT>::iterateEvolvingSet (float flow_r)
{
  std::vector<std::pair<float, int> > active_pixel;
  size_t num_superpixels = superpixels_.size ();
  for (int i = 0; i<num_superpixels; ++i)
  {
    std::list<std::vector<std::pair<float, int> > >::iterator itr = evolving_set_[i].begin();
    std::list<std::vector<std::pair<float, int> > >::iterator itr_end = evolving_set_[i].end();
    while ( itr != itr_end )
    {
      int num_active = (*itr).size ();
      for (int k = 0; k < (*itr).size (); ++k)
      {
        //If this neighbor distance is still > 0
        if ((*itr)[k].first > 0)
        {
          (*itr)[k].first -= flow_r;
          if ((*itr)[k].first <= 0)
          {
            //Check to make sure this neighbor hasn't been assigned already
            int neighbor_idx = (*itr)[k].second;
            if (point_labels_[neighbor_idx] == -1)
            {
              //Label it
              point_labels_[neighbor_idx] = i;
              superpixels_[i].indices.push_back (neighbor_idx);
              //Add its neighbors to this evolving set
              active_pixel = point_dist_neighbor_[neighbor_idx];
              evolving_set_[i].push_back (active_pixel);
              //TODO: Do we adjust end here, or just wait until next iteration to work?
            }
            //Decriment number of active neighbors
            num_active--;
          }
        }
        else //else this neighbor is already dead
        {
          num_active--;
        }
      }
      //If this pixel has no more active neighbors, then we can kill it
      if (num_active == 0)
      {
        evolving_set_[i].erase (itr++);
      }
      else
      {
        ++itr;
      }
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
  std::cout << "Getting colored cloud - superpixels size = "<<superpixels_.size()<<std::endl;
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
        uint8_t r = (static_cast<unsigned char> (rand () % 200) + 50);
        uint8_t g = (static_cast<unsigned char> (rand () % 256));
        uint8_t b = (static_cast<unsigned char> (rand () % 256));
        superpixel_colors_.push_back ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      }
    }
    pcl::copyPointCloud (*input_,*colored_cloud);

    pcl::PointCloud <pcl::PointXYZRGB>::iterator i_colored;
    typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin ();
    int num_points_not_in_voxel = 0;
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
          //std::cout << "Nearest neighbor sq="<<sqr_dist<< " which is bigger then res sq="<<resolution_*resolution_<<std::endl;
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
    std::cout<<"Num points not in a voxel:"<<num_points_not_in_voxel<<std::endl;
  }
  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZL>::Ptr
pcl::VoxelSuperpixels<PointT>::getLabeledCloud ()
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud;
  std::cout << "Getting labeled cloud - superpixels size = "<<superpixels_.size()<<std::endl;
  if (!superpixels_.empty ())
  {
    labeled_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZL> > ();

    pcl::copyPointCloud (*input_,*labeled_cloud);

    pcl::PointCloud <pcl::PointXYZL>::iterator i_labeled;
    typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin ();
    int next_color = 0;
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
  
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud (voxel_cloud_);
  
  if (voxel_kdtree_ != 0)
    voxel_kdtree_.reset ();
  
  voxel_kdtree_ = boost::make_shared< pcl::search::KdTree<PointT> >();
  ne.setSearchMethod (voxel_kdtree_);
  
  normals_ = boost::make_shared< pcl::PointCloud<pcl::Normal> >();
  
  ne.setRadiusSearch (resolution_*5);
  ne.compute (*normals_);
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
    
  //  std::cout << "X="<<X<< "  Y="<<Y<<"  Z="<<Z<<std::endl;
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
  //  std::cout << "L="<<voxel_LAB_[i][0]<< "  a="<<voxel_LAB_[i][1]<<"  b="<<voxel_LAB_[i][2]<<std::endl;
  }
  
  
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::VoxelSuperpixels<PointT>::calcColorDifference (PointT a, PointT b)
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
  return (difference);
  
  
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
    uint32_t green = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    r = 255; g = 0;
    uint32_t red = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    r = 0; b = 255;
    uint32_t blue = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    std::vector<int> neighbors;
    std::vector<float> sqr_distances;

    typename pcl::search::KdTree<PointT>::Ptr orig_kdtree = boost::make_shared< pcl::search::KdTree<PointT> >();
    orig_kdtree->setInputCloud (input_);

      
    for (int i = 0; i < seed_indices_orig_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_orig_[i]];
      int num = orig_kdtree->radiusSearch (point, 0.4*seed_resolution_, neighbors, sqr_distances);
      for (int j = 0; j < num; ++j)
      {
        seed_cloud->points[neighbors[j]].rgb = *reinterpret_cast<float*>(&red);
      }
    }
    
    for (int i = 0; i < seed_indices_.size (); ++i)
    {
      PointT point = voxel_cloud_->points[seed_indices_[i]];
      int num = orig_kdtree->radiusSearch (point, 0.4*seed_resolution_, neighbors, sqr_distances);
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
 
