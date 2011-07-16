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
 *
 *
 */

#ifndef PCL_SEARCH_GENERIC_SEARCH_IMPL
#define PCL_SEARCH_GENERIC_SEARCH_IMPL

#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"
#include "pcl/search/octree_pointcloud.h"
#include "pcl/search/organized_neighbor_search.h"

using namespace std;
namespace pcl
{



template <typename PointT> void 
Search<PointT>::setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr &indices)
{

}

template <typename PointT> void 
Search<PointT>::setInputCloud (const PointCloudConstPtr& cloud)
{

}

template <typename PointT> int 
Search<PointT>::nearestKSearch (const PointT& point,  int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances) 
{

}


template <typename PointT> int 
Search<PointT>::nearestKSearch (const PointCloud& cloud, int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances) 
{
}



template <typename PointT> int 
Search<PointT>::nearestKSearch (int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances)
{
}


template <typename PointT> int 
Search<PointT>::radiusSearch (const PointT& point, const double radius, std::vector<int>& k_indices, std::vector<float>& k_distances, int max_nn) const
{
}

template <typename PointT> int 
Search<PointT>::radiusSearch (const PointCloud& cloud, int index, double radius,
                              std::vector<int>& k_indices, std::vector<float>& k_distances,
                              int max_nn) 
{
}

template <typename PointT> int 
Search<PointT>::radiusSearch (int index, double radius, std::vector<int>& k_indices,
                              std::vector<float>& k_distances, int max_nn) const
{
}

template <typename PointT> void 
        Search<PointT>::
        approxNearestSearch (const PointCloudConstPtr &cloud_arg, int query_index_arg, int &result_index_arg,
                             float &sqr_distance_arg){}

template <typename PointT> void 
        Search<PointT>::
        approxNearestSearch (const PointT &p_q_arg, int &result_index_arg, float &sqr_distance_arg){};

template <typename PointT> void 
        Search<PointT>::
        approxNearestSearch (int query_index_arg, int &result_index_arg, float &sqr_distance_arg){};

template <typename PointT> int
Search<PointT>::approxRadiusSearch (const PointCloud& cloud, int index, double radius,
                              std::vector<int>& k_indices, std::vector<float>& k_distances,
                              int max_nn)
{
}


template <typename PointT> int
Search<PointT>::approxNearestKSearch (const PointCloud& cloud, int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances)
{
}

template <typename PointT> void 
            Search<PointT>::evaluateSearchMethods (const PointCloudConstPtr& cloud, const int search_type){}

}
#define PCL_INSTANTIATE_Search(T) template class PCL_EXPORTS pcl::Search<T>;

#endif  //#ifndef PCL_SEARCH_GENERIC_SEARCH_IMPL
