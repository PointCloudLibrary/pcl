/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *  Author: Siddharth Choudhary (itzsid@gmail.com)
 *
 */

#ifndef PCL_SEARCH_AUTO_TUNED_SEARCH_IMPL
#define PCL_SEARCH_AUTO_TUNED_SEARCH_IMPL

#include <pcl/pcl_base.h>
#include <vector>
#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"
#include "pcl/search/octree_pointcloud.h"
#include "pcl/search/organized_neighbor_search.h"
#include <pcl/common/time.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::AutotunedSearch<PointT>::initSearchDS (int spatial_locator)
{
  // specifies the data-structure to be used using spatial_locator flag
  if (spatial_locator == KDTREE_FLANN)
    // initialize kdtree
    search_.reset (new KdTreeWrapper<PointT> ());
  else if (spatial_locator == ORGANIZED_INDEX)
    search_.reset (new OrganizedNeighborSearch<PointT> ());
  else if (spatial_locator == OCTREE)
    search_.reset (new pcl::octree::OctreeWrapper<PointT> (0.1f));
  else
    PCL_ERROR ("[pcl::AutotunedSearch::initSearchDS] Spatial locator (%d) unknown!\n", spatial_locator);
   spatial_loc_ = spatial_locator;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::AutotunedSearch<PointT>::evaluateSearchMethods (const PointCloudConstPtr& cloudIn, const int search_type)
{
  // Evaluates different search data-structures for the given data and sets it to the ds having minimum time
	unsigned int searchIdx;
  while (1)
  {
    searchIdx = rand()%(cloudIn->width * cloudIn->height);
    if (cloudIn->points[searchIdx].z < 100)
      break;
   }
  const PointT& searchPoint = cloudIn->points[searchIdx];

  double time_kdtree, time_octree, time_organized_data;
  if (search_type == NEAREST_K_SEARCH)
  {
    unsigned int no_of_neighbors = 20;
    std::vector<int> k_indices;
    k_indices.resize (no_of_neighbors);
    std::vector<float> k_distances;
    k_distances.resize (no_of_neighbors);
    std::cout << "\n---------------\nKDTree\n---------------\n";
    double time1 = getTime ();
    search_.reset (new KdTreeWrapper<PointT> ());
    search_->setInputCloud (cloudIn);
//    search_->nearestKSearch (searchPoint, no_of_neighbors, k_indices, k_distances);
    std::cout << "Neighbors are:" << std::endl;
/*    
    for (int i = 0; i < 20; i++)
    {
      std::cout << k_indices[i] << '\t'; 
    }
    std::cout << std::endl;
    std::cout << "Number of Neighbors: " << k_indices.size () << std::endl; 
    k_indices.clear (); k_distances.clear ();

    std::cout << "\n---------------\nOrganizedData\n---------------\n";
    double time2 = getTime ();
    search_.reset (new OrganizedNeighborSearch<PointT>());
    search_->setInputCloud (cloudIn);
    search_->nearestKSearch (searchPoint, no_of_neighbors, k_indices, k_distances);
    std::cout << "Neighbors are: " << std::endl;
    for (int i = 0;i < 20; i++)
    {
      std::cout << k_indices[i] << '\t'; 
    }
    std::cout << std::endl;
    std::cout << "Number of Neigbhors: " << k_indices.size () << std::endl; 
    k_indices.clear (); k_distances.clear ();

    std::cout << "\n---------------\nOctree\n---------------\n";
    double time3 = getTime ();
    search_.reset (new pcl::octree::OctreeWrapper<PointT> (0.1f));
    search_->setInputCloud (cloudIn);
    search_->nearestKSearch (searchPoint, no_of_neighbors, k_indices, k_distances);
    std::cout << "Neighbors are: " << std::endl;
    for (int i = 0;i < 20; i++)
    {
      std::cout << k_indices[i] << '\t'; 
    }
    std::cout << std::endl;
    std::cout << "Number of Neighbors: " << k_indices.size () << std::endl; 
    k_indices.clear (); k_distances.clear ();
    std::cout << std::endl;
    std::cout << std::endl;

    time_kdtree = time2 - time1;
    time_organized_data = time3 - time2;
    time_octree = getTime () - time3;
    
    std::cout << "Time Taken: " << "KDTree: " << time_kdtree << '\t' <<"OranizedData: "  << time_organized_data << '\t' << "Octree: " << time_octree << '\t' << std::endl; */
  }
/*  else if(search_type == NEAREST_RADIUS_SEARCH)
  {

       double searchRadius = 1.0 * ((double)rand () / (double)RAND_MAX);

       std::vector<int> k_indices;
       std::vector<float> k_distances;
          std::cout << "\n---------------\nKDTree\n---------------\n";
          double time1 = getTime();
          search_.reset(new KdTreeWrapper<PointT>());
          search_->setInputCloud(cloudIn);
          search_->radiusSearch (searchPoint, searchRadius, k_indices, k_distances);
          std::cout << "Neighbors are:" << std::endl;

          for(int i=0;i<20;i++){
          std::cout << k_indices[i] << '\t';
          }
          std::cout << std::endl;
          std::cout << "Number of Neighbors: " << k_indices.size() << std::endl; k_indices.clear();k_distances.clear();

          std::cout << "\n---------------\nOrganizedData\n---------------\n";
          double time2 = getTime();
          search_.reset(new OrganizedNeighborSearch<PointT>());
          search_->setInputCloud(cloudIn);
          search_->radiusSearch (searchPoint, searchRadius, k_indices, k_distances);
          std::cout << "Neighbors are: " << std::endl;
          for(int i=0;i<20;i++){
          std::cout << k_indices[i] << '\t';
          }
          std::cout << std::endl;
          std::cout << "Number of Neigbhors: " << k_indices.size() << std::endl; k_indices.clear();k_distances.clear();

          std::cout << "\n---------------\nOctree\n---------------\n";
          double time3 = getTime();
          search_.reset(new pcl::octree::OctreeWrapper<PointT>(0.1f));
          search_->setInputCloud(cloudIn);
          search_->radiusSearch (searchPoint, searchRadius, k_indices, k_distances);
          std::cout << "Neighbors are: " << std::endl;
          for(int i=0;i<20;i++){
          std::cout << k_indices[i] << '\t';
          }
          std::cout << std::endl;
          std::cout << "Number of Neighbors: " << k_indices.size() << std::endl; k_indices.clear();k_distances.clear();
          std::cout << std::endl;
          std::cout << std::endl;


     time_kdtree = time2 - time1;
     time_organized_data = time3 - time2;
     time_octree = getTime() - time3;
    
    std::cout << "Time Taken: " << "KDTree: " << time_kdtree << '\t' <<"OranizedData: "  << time_organized_data << '\t' << "Octree: " << time_octree << '\t' << std::endl;	

  }
  else
  {

  std::cerr << "Only NEAREST_K_SEARCH and NEAREST_RADIUS_SEARCH supported" << std::endl;
  exit(0);
  }
  // Set the datastructure according to which takes the minimum time
  if(time_kdtree < time_organized_data && time_kdtree < time_octree)
  {
  spatial_loc_ = KDTREE_FLANN;
  search_.reset(new KdTreeWrapper<PointT>());
  }
  else if(time_octree < time_kdtree && time_octree < time_organized_data)
  {
  spatial_loc_ = OCTREE;
  search_.reset(new pcl::octree::OctreeWrapper<PointT>(0.1f));
  }
  else if(time_organized_data < time_kdtree && time_organized_data < time_octree)
  {
  spatial_loc_ = OCTREE;
          search_.reset(new OrganizedNeighborSearch<PointT>());
  }
*/
//  search_->setInputCloud (cloudIn);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::AutotunedSearch<PointT>::setInputCloud (const PointCloudConstPtr &cloud, 
                                             const IndicesConstPtr &indices)
{
  input_ = cloud;
  search_->setInputCloud (cloud, indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::AutotunedSearch<PointT>::setInputCloud (const PointCloudConstPtr& cloud)
{
  input_ = cloud;
  search_->setInputCloud (cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::AutotunedSearch<PointT>::nearestKSearch (const PointT &point,
                                              int k, 
                                              std::vector<int> &k_indices, 
                                              std::vector<float> &k_sqr_distances) 
{
  //return (search_->nearestKSearch (point, k, k_indices, k_sqr_distances));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::AutotunedSearch<PointT>::nearestKSearch (const PointCloud &cloud, 
                                              int index, 
                                              int k, 
                                              std::vector<int> &k_indices, 
                                              std::vector<float> &k_sqr_distances) 
{
  return (search_->nearestKSearch (cloud, index, k, k_indices, k_sqr_distances));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::AutotunedSearch<PointT>::nearestKSearch (int index, 
                                              int k, 
                                              std::vector<int> &k_indices, 
                                              std::vector<float> &k_sqr_distances)
{
  return (search_->nearestKSearch (index, k, k_indices, k_sqr_distances));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::AutotunedSearch<PointT>::radiusSearch (const PointT &point, 
                                            const double radius, 
                                            std::vector<int> &k_indices, 
                                            std::vector<float> &k_distances, 
                                            int max_nn) const
{
  return (search_->radiusSearch (point, radius, k_indices, k_distances, max_nn));
}
    
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::AutotunedSearch<PointT>::radiusSearch (const PointCloud &cloud, 
                                            int index, 
                                            double radius,
                                            std::vector<int> &k_indices, 
                                            std::vector<float> &k_distances,
                                            int max_nn) 
{
  return (search_->radiusSearch (cloud, index, radius, k_indices, k_distances, max_nn));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
pcl::AutotunedSearch<PointT>::radiusSearch (int index, 
                                            double radius, 
                                            std::vector<int> &k_indices,
                                            std::vector<float> &k_distances, 
                                            int max_nn) const
{
  return (search_->radiusSearch (index, radius, k_indices, k_distances, max_nn));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::AutotunedSearch<PointT>::approxNearestSearch (const PointCloudConstPtr &cloud_arg, 
                                                   int query_index_arg, 
                                                   int &result_index_arg,
                                                   float &sqr_distance_arg)
{
  if(spatial_loc_ == OCTREE)
    search_->approxNearestSearch (cloud_arg, query_index_arg, result_index_arg, sqr_distance_arg);
  else
    PCL_ERROR ("approxNearestSearch() works only for OCTREE structure\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::AutotunedSearch<PointT>::approxNearestSearch (const PointT &p_q_arg, 
                                                   int &result_index_arg, 
                                                   float &sqr_distance_arg)
{
  if (spatial_loc_ == OCTREE)
    search_->approxNearestSearch (p_q_arg, result_index_arg, sqr_distance_arg);
  else
    PCL_ERROR ("approxNearestSearch() works only for OCTREE structure\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::AutotunedSearch<PointT>::approxNearestSearch (int query_index_arg, 
                                                   int &result_index_arg, 
                                                   float &sqr_distance_arg)
{
  if (spatial_loc_ == OCTREE)
   search_->approxNearestSearch (query_index_arg, result_index_arg, sqr_distance_arg);
  else
    PCL_ERROR ("approxNearestSearch() works only for OCTREE structure\n");
}

#define PCL_INSTANTIATE_AutotunedSearch(T) template class PCL_EXPORTS pcl::AutotunedSearch<T>;

#endif  //#ifndef PCL_SEARCH_AUTO_TUNED_SEARCH_IMPL
