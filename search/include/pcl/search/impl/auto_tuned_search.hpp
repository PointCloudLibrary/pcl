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

#ifndef PCL_SEARCH_AUTO_TUNED_SEARCH_IMPL
#define PCL_SEARCH_AUTO_TUNED_SEARCH_IMPL

#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"
#include "pcl/search/octree_pointcloud.h"
#include "pcl/search/organized_neighbor_search.h"

using namespace std;
namespace pcl
{

template <typename PointT> void 
AutotunedSearch<PointT>::initSearchDS (int spatial_locator)
{
        #if 1

    if(spatial_locator == KDTREE_FLANN) {
        // initialize kdtree
        _searchptr.reset(new KdTree<PointT>());
    }
    else if(spatial_locator == ORGANIZED_INDEX) {
        _searchptr.reset(new OrganizedNeighborSearch<PointT>());
    }
    else if(spatial_locator == OCTREE) {

        printf("OCTREE NOT YET IMPLEMENTED");
	exit(0);
	
    }

   spatial_loc = spatial_locator;

    #endif

}


template <typename PointT> void 
AutotunedSearch<PointT>::setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr &indices)
{

    /* for kdtree */
    _searchptr->setInputCloud(cloud, indices);



}

template <typename PointT> void 
AutotunedSearch<PointT>::setInputCloud (const PointCloudConstPtr& cloud)
{
    _searchptr->setInputCloud(cloud);

}

template <typename PointT> int 
AutotunedSearch<PointT>::nearestKSearch (const PointT& point,  int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances) 
{

    return _searchptr->nearestKSearch(point,k,k_indices,k_sqr_distances);

}


template <typename PointT> int 
AutotunedSearch<PointT>::nearestKSearch (const PointCloud& cloud, int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances) 
{
    return _searchptr->nearestKSearch(cloud,index,k,k_indices,k_sqr_distances);
}



template <typename PointT> int 
AutotunedSearch<PointT>::nearestKSearch (int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances)
{
    return _searchptr->nearestKSearch(index,k,k_indices,k_sqr_distances);
}


template <typename PointT> int 
AutotunedSearch<PointT>::radiusSearch (const PointT& point, const double radius, std::vector<int>& k_indices, std::vector<float>& k_distances, int max_nn) const
{

    return _searchptr->radiusSearch(point,radius,k_indices,k_distances,max_nn);
}

template <typename PointT> int 
AutotunedSearch<PointT>::radiusSearch (const PointCloud& cloud, int index, double radius,
                              std::vector<int>& k_indices, std::vector<float>& k_distances,
                              int max_nn) 
{
    return _searchptr->radiusSearch(cloud,index,radius,k_indices,k_distances,max_nn);
}

template <typename PointT> int 
AutotunedSearch<PointT>::radiusSearch (int index, double radius, std::vector<int>& k_indices,
                              std::vector<float>& k_distances, int max_nn) const
{

    return _searchptr->radiusSearch(index,radius,k_indices,k_distances,max_nn);
}

template <typename PointT> void 
        AutotunedSearch<PointT>::
        approxNearestSearch (const PointCloudConstPtr &cloud_arg, int query_index_arg, int &result_index_arg,
                             float &sqr_distance_arg){}

template <typename PointT> void 
        AutotunedSearch<PointT>::
        approxNearestSearch (const PointT &p_q_arg, int &result_index_arg, float &sqr_distance_arg){};

template <typename PointT> void 
        AutotunedSearch<PointT>::
        approxNearestSearch (int query_index_arg, int &result_index_arg, float &sqr_distance_arg){};


template <typename PointT> void 
AutotunedSearch<PointT>::setMethod(int k)
{
    _searchptr->setMethod(k);
}

template <typename PointT> void 
       AutotunedSearch<PointT>::deleteTree ( bool freeMemory_arg)
{

}

template <typename PointT> void 
        AutotunedSearch<PointT>::
        addPointsFromInputCloud ()
{
}

}
#define PCL_INSTANTIATE_AutotunedSearch(T) template class PCL_EXPORTS pcl::AutotunedSearch<T>;

#endif  //#ifndef PCL_SEARCH_AUTO_TUNED_SEARCH_IMPL
