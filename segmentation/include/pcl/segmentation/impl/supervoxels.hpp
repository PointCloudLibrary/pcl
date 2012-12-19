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
 * Author : jpapon@gmail.com
 * Email  : jpapon@gmail.com
 *
 */

#ifndef PCL_SEGMENTATION_SUPERVOXELS_HPP_
#define PCL_SEGMENTATION_SUPERVOXELS_HPP_

#include <pcl/segmentation/supervoxels.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/octree/octree_impl.h>





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::SuperVoxels<PointT>::SuperVoxels (float voxel_resolution, float seed_resolution) :
  resolution_ (voxel_resolution),
  seed_resolution_ (seed_resolution),
  voxel_cloud_ (),
  seed_indices_ (0),
  voxel_kdtree_ (),
  voxel_octree_ (),
  seed_octree_ (),
  normal_radius_(resolution_*3.0f),
  color_importance_(0.1f),
  spatial_importance_(0.4f),
  normal_importance_(1.0f),
  voxel_fpfh_(),
  seed_indices_orig_ (0),
  seed_indices_unshifted_ (0),
  label_colors_ (0)

{
  label_colors_.reserve (MAX_LABEL);
  label_colors_.push_back (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));
  
  srand (static_cast<unsigned int> (time (0)));
  for (size_t i_segment = 0; i_segment < MAX_LABEL - 1; i_segment++)
  {
    uint8_t r = static_cast<uint8_t>( (rand () % 256));
    uint8_t g = static_cast<uint8_t>( (rand () % 256));
    uint8_t b = static_cast<uint8_t>( (rand () % 256));
    label_colors_.push_back (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::SuperVoxels<PointT>::~SuperVoxels ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::SuperVoxels<PointT>::getNormalRadius () const
{
  return (normal_radius_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::setNormalRadius (float radius)
{
  normal_radius_ = radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::SuperVoxels<PointT>::getVoxelResolution () const
{
  return (resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::setVoxelResolution (float resolution)
{
  resolution_ = resolution;
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::SuperVoxels<PointT>::getSeedResolution () const
{
  return (resolution_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::setSeedResolution (float seed_resolution)
{
  seed_resolution_ = seed_resolution;
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::SuperVoxels<PointT>::getVoxelCloudSize () const
{
  if (voxel_cloud_)
    return voxel_cloud_->width;
  else
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<pcl::PointSuperVoxel>::Ptr
pcl::SuperVoxels<PointT>::getVoxelCloud ()
{
  return voxel_cloud_;
}
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::setColorImportance (float val)
{
  color_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::setSpatialImportance (float val)
{
  spatial_importance_ = val;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::setNormalImportance (float val)
{
  normal_importance_ = val;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::extract (typename pcl::PointCloud<PointSuperVoxel>::Ptr &voxel_cloud)
{
  timer_.reset ();

  double t_start = timer_.getTime ();
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
  double t_prep = timer_.getTime ();
  std::cout << "Placing Seeds" << std::endl;
  placeSeedVoxels ();
  double t_seeds = timer_.getTime ();
 
  std::cout << "Evolving the superpixels" << std::endl;
  evolveSuperpixels ();
  double t_iterate = timer_.getTime ();
  
  voxel_octree_->getCentroidCloud (voxel_cloud_);
  double t_get_cloud = timer_.getTime ();
  
  voxel_octree_->computeSuperVoxelAdjacencyGraph (label_centers_,supervoxel_adjacency_graph_);
  double t_create_graph = timer_.getTime();
   
  if (!voxel_cloud)
    voxel_cloud = boost::make_shared<pcl::PointCloud<PointSuperVoxel> >();
  pcl::copyPointCloud (*voxel_cloud_, *voxel_cloud);
  double t_copy_out = timer_.getTime();
  
  std::cout << "--------------------------------- Timing Report --------------------------------- \n";
  std::cout << "Time to prep (normals, neighbors, voxelization)="<<t_prep-t_start<<" ms\n";
  std::cout << "Time to seed clusters                          ="<<t_seeds-t_prep<<" ms\n";
  std::cout << "Time to expand clusters                        ="<<t_iterate-t_seeds<<" ms\n";
  std::cout << "Time to extract voxel cloud from octree        ="<<t_get_cloud-t_iterate<<" ms\n";
  std::cout << "Time to create super voxel adjacency graph     ="<<t_create_graph-t_get_cloud<<" ms\n";
  std::cout << "Time to copy out final voxel cloud             ="<<t_copy_out-t_create_graph<<" ms\n";
  std::cout << "Total run time                                 ="<<t_copy_out-t_start<<" ms\n";
  std::cout << "--------------------------------------------------------------------------------- \n";
  
  deinitCompute ();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::getSuperVoxelAdjacencyList (VoxelAdjacencyList &adjacency_list_arg)
{
  voxel_octree_->computeSuperVoxelAdjacencyGraph (label_centers_, adjacency_list_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::getSuperVoxelCenters (std::map<uint32_t, PointSuperVoxel> &label_centers_arg)
{
  label_centers_arg = label_centers_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::SuperVoxels<PointT>::getColoredCloud ()
{
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = boost::make_shared <pcl::PointCloud<pcl::PointXYZRGB> >();
  pcl::copyPointCloud (*input_,*colored_cloud);
  pcl::PointCloud <pcl::PointXYZRGB>::iterator i_colored;
  typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin ();
  std::vector <int> indices;
  std::vector <float> sqr_distances;
  for (i_colored = colored_cloud->begin (); i_colored != colored_cloud->end (); ++i_colored,++i_input)
  {
    i_colored->rgb = 0; 
    if ( !pcl::isFinite<PointT> (*i_input))
      i_colored->rgb = 0;
    else
    {     
      pcl::PointSuperVoxel temp;
      temp.x = i_input->x; temp.y = i_input->y; temp.z = i_input->z;
      voxel_kdtree_->nearestKSearch (temp, 1, indices, sqr_distances);
      if (indices.size () > 0)
        i_colored->rgb = *reinterpret_cast<float*>(&label_colors_[voxel_cloud_->points[indices[0]].label]);
    }
    
  }
  
  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::SuperVoxels<PointT>::getColoredVoxelCloud ()
{
  voxel_octree_->getCentroidCloud (voxel_cloud_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  colored_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> > ();
  pcl::copyPointCloud (*voxel_cloud_,*colored_cloud);
  pcl::PointCloud <pcl::PointXYZRGB>::iterator i_colored;
  typename pcl::PointCloud <pcl::PointSuperVoxel>::const_iterator i_voxel = voxel_cloud_->begin ();
  for (i_colored = colored_cloud->begin (); i_colored != colored_cloud->end (); ++i_colored,++i_voxel)
  {
    i_colored->rgb = *reinterpret_cast<float*>(&label_colors_ [i_voxel->label]);
  }
  
  return colored_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZL>::Ptr
pcl::SuperVoxels<PointT>::getLabeledVoxelCloud ()
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud;
  if (!voxel_cloud_->empty ())
  {
    labeled_voxel_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZL> > ();
    
    pcl::copyPointCloud (*voxel_cloud_,*labeled_voxel_cloud);
  }
  
  return labeled_voxel_cloud;  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZL>::Ptr
pcl::SuperVoxels<PointT>::getLabeledCloud ()
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud;
  if (!voxel_cloud_->empty ())
  {
    labeled_cloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZL> > ();
    pcl::copyPointCloud (*input_,*labeled_cloud);
    pcl::PointCloud <pcl::PointXYZL>::iterator i_labeled;
    std::vector <int> indices;
    std::vector <float> sqr_distances;
    typename pcl::PointCloud <PointT>::const_iterator i_input = input_->begin ();
    for (i_labeled = labeled_cloud->begin (); i_labeled != labeled_cloud->end (); ++i_labeled, ++i_input)
    {
      i_labeled->label = 0;
      if ( pcl::isFinite<PointT> (*i_input))
      {  
        pcl::PointSuperVoxel temp;
        temp.x = i_input->x; temp.y = i_input->y; temp.z = i_input->z;
        voxel_kdtree_->nearestKSearch (temp, 1, indices, sqr_distances);
        i_labeled->label = voxel_cloud_->points[indices[0]].label;
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
pcl::SuperVoxels<PointT>::prepareForSegmentation ()
{

  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
    return (false);

  double prep_start = timer_.getTime ();
  std::cout << "Populating voxel octree using input cloud \n";
  voxel_octree_ = boost::make_shared<pcl::octree::OctreePointCloudSuperVoxel<PointT> >(resolution_, seed_resolution_, color_importance_, normal_importance_, spatial_importance_);
  voxel_octree_->setInputCloud(input_);
  voxel_octree_->defineBoundingBox(); 
  voxel_octree_->addPointsFromInputCloud ();
  double prep_end = timer_.getTime ();
  std::cout<<"Time elapsed populating octree ="<<prep_end-prep_start<<" ms\n";
  
  voxel_octree_->getCentroidCloud (voxel_cloud_);
  //This computes an octree with seed resolution that indexes the voxel cloud
  seed_octree_ =  boost::make_shared<pcl::octree::OctreePointCloudSearch <pcl::PointSuperVoxel> > (seed_resolution_);
  seed_octree_->setInputCloud (voxel_cloud_);
  seed_octree_->addPointsFromInputCloud ();
    
  std::cout << "Computing Normals (and Kd tree)\n";
  // Compute normals for voxel cloud
  double normals_start = timer_.getTime ();
  computeNormals ();
  voxel_octree_->insertNormals (voxel_cloud_->points);
  double normals_end = timer_.getTime ();
  std::cout<<"Time elapsed compute voxel normals, insert into octree="<<normals_end-normals_start<<" ms\n";
  
  double neighbors_start = timer_.getTime ();
  voxel_octree_->computeNeighbors ();
  double neighbors_end = timer_.getTime ();
  std::cout<<"Time elapsed compute voxel neighbors="<<neighbors_end-neighbors_start<<" ms\n";
  
  //TODO Test whether LAB actually helps or not on NYU dataset
  //std::cout << "Computing Lab values \n";
  //calcVoxelLABValues();
  
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::placeSeedVoxels ()
{
  
  std::vector<PointSuperVoxel, Eigen::aligned_allocator<PointSuperVoxel> > voxel_centers; 
  int num_seeds = seed_octree_->getOccupiedVoxelCenters(voxel_centers); 
  std::cout << "Number of seed points before filtering="<<num_seeds<<std::endl;
  
  seed_indices_orig_.resize (num_seeds, 0);
  std::vector<int> closest_index;
  std::vector<float> distance;
  closest_index.resize(1,0);
  distance.resize(1,0);
  for (int i = 0; i < num_seeds; ++i)  
  {
    PointSuperVoxel center_point = voxel_centers[i];
    voxel_kdtree_->nearestKSearch (center_point, 1, closest_index, distance);
    seed_indices_orig_[i] = closest_index[0];
  }

  std::vector<int> neighbors;
  std::vector<float> sqr_distances;

  float search_radius = 0.3f*seed_resolution_;
 // float merge_radius = 2.0f * resolution_;
  //std::cout << "Search radius = "<<search_radius<<"\n";
  // This is number of voxels which fit in a planar slice through search volume
  // Area of planar slice / area of voxel side
  float min_points = 0.5f * (search_radius)*(search_radius) * 3.1415926536f  / (resolution_*resolution_);
  for (int i = 0; i < seed_indices_orig_.size (); ++i)
  {
    int num = voxel_kdtree_->radiusSearch (seed_indices_orig_[i], search_radius , neighbors, sqr_distances);
  //  std::cout << "Found " << neighbors.size () << " points within radius "<<search_radius<<std::endl;
  //  float min_gradient = calcGradient(seed_indices_orig_[i]);
    int min_index = seed_indices_orig_[i];
    //TODO ENABLE GRADIENT?
 //   float neighbor_gradient;
  //  for (int j = 1; j < num; ++j)
  //  {
  //    if (sqr_distances[j] < (search_radius*search_radius))
  //    {
 //       neighbor_gradient = calcGradient(neighbors[j]);
 //       if (neighbor_gradient < min_gradient) 
 //       {
  //        min_gradient = neighbor_gradient;
   //       min_index = neighbors[j];
    //    }
   //   }
   // }
    
    //TODO Re-enable merging of seeds that are very close?
    //Check to see if there are any seeds already here, if so, don't add this seed
    //voxel_kdtree_->radiusSearch (min_index, merge_radius, neighbors, sqr_distances);
    bool add_seed = true;
  //  for (int k = 0; k < neighbors.size (); ++k)
  //  {
   //   for (int j = 0; j < seed_indices_.size (); ++j)
     // {
  //      if ( seed_indices_[j] == neighbors[k] ) //TODO CHECK color sim, if far, add seed anyways
     //   {  
    //      add_seed = false;
          //std::cout << "Duplicate, removing seed index "<<i<<"\n";
    //      break;
   //     }
    //  }
    //  if (!add_seed)
   //     break;
   // }

    if ( num > min_points && add_seed)
    {
      seed_indices_.push_back (min_index);
      //seed_indices_.push_back (seed_indices_orig_[i]);
      //seed_indices_unshifted_.push_back (seed_indices_orig_[i]);
    }
    
  }
  std::cout << "Number of seed points after filtering="<<seed_indices_.size ()<<std::endl;
  
  //pcl::PointIndices superpixel;
  //superpixels_.resize(seed_indices_.size (), superpixel);
  //for (int i = 0; i < seed_indices_.size (); ++i)
  //  superpixels_[i].indices.push_back (seed_indices_[i]);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::evolveSuperpixels ()
{
  //Calculate initial values for clusters
  std::cout << "Initializing Superpixel Clusters...\n";
  initSuperpixelClusters ();
   
  int max_depth = static_cast<int>(seed_resolution_/resolution_);
  std::cout << "Max depth = "<<max_depth<<"\n";
  
  for (int i = 1; i < max_depth; ++i)
  {
      //Expand the the labels by one iteration
      voxel_octree_->iterateVoxelLabels (label_centers_);
      //Update the centers to reflect new centers
      voxel_octree_->updateCenters (label_centers_); 
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::initSuperpixelClusters ()
{
  size_t num_seeds = seed_indices_.size ();
  for (int i=0; i<num_seeds; ++i)
  {
    uint32_t label = static_cast<uint32_t>(i+1);
    PointSuperVoxel center_point = voxel_octree_->setPointLabel (voxel_cloud_->points[seed_indices_[i]],label);
    label_centers_.insert (LabelCenterT (label,center_point));
  }
    
  //Expand the the labels by one iteration
  voxel_octree_->iterateVoxelLabels (label_centers_);
  //Update the centers to reflect new centers
  voxel_octree_->updateCenters (label_centers_); 

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::computeNormals ()
{
  pcl::NormalEstimationOMP<pcl::PointSuperVoxel, pcl::PointSuperVoxel> ne;
  ne.setInputCloud (voxel_cloud_);
  
  if (voxel_kdtree_ != 0)
    voxel_kdtree_.reset ();
  
  voxel_kdtree_ = boost::make_shared< pcl::search::KdTree<pcl::PointSuperVoxel> >();
  ne.setSearchMethod (voxel_kdtree_);
  ne.setRadiusSearch (normal_radius_);
  ne.compute (*voxel_cloud_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SuperVoxels<PointT>::calcVoxelLABValues ()
{
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
    
  //  (voxel_cloud_->points[i]).L = (116.0f * Y) - 16.0f;
  //  (voxel_cloud_->points[i]).A = 500.0f * ( X - Y );
 //   (voxel_cloud_->points[i]).B = 200.0f * ( Y - Z );
  }
  
  
}


#endif    // PCL_SEGMENTATION_SUPERVOXELS_HPP_
 
