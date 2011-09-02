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

#ifndef PCL_SEARCH_AUTO_TUNED_SEARCH_H_
#define PCL_SEARCH_AUTO_TUNED_SEARCH_H_

#include <limits.h>
#include <pcl/pcl_base.h>
#include "pcl/pcl_macros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl/search/search.h"

namespace pcl
{
  namespace search
  {
    enum SearchType
    {
      KDTREE,
      ORGANIZED_INDEX,
      OCTREE,
      AUTO_TUNED,
      NEAREST_K_SEARCH,
      NEAREST_RADIUS_SEARCH
    };

    template <typename PointT>
    class AutotunedSearch : public pcl::search::Search<PointT>
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        typedef boost::shared_ptr<pcl::search::Search<PointT> > SearchPtr;
        typedef boost::shared_ptr<const pcl::search::Search<PointT> > SearchConstPtr;

        typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;

        AutotunedSearch (int spatial_locator) : search_ ()
        {
          initSearchDS (spatial_locator);
        }

        AutotunedSearch () : search_ () {}

        virtual ~AutotunedSearch () {}

        void 
        initSearchDS (int spatial_locator);

        void
        evaluateSearchMethods (const PointCloudConstPtr& cloud, const int search_type);

        virtual void 
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices);

        virtual void 
        setInputCloud (const PointCloudConstPtr& cloud);

        int
        nearestKSearch (const PointT& point, 
                        int k, 
                        std::vector<int> &k_indices, 
                        std::vector<float> &k_sqr_distances);

        virtual int
        nearestKSearch (const PointCloud& cloud, int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances);

        virtual int
        nearestKSearch (int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances);

        inline int
        nearestKSearch (std::vector<PointT, Eigen::aligned_allocator<PointT> >& point, 
                        std::vector <int>& k, 
                        std::vector<std::vector<int> >& k_indices,
                        std::vector<std::vector<float> >& k_sqr_distances)
        {
          PCL_ERROR("[pcl::search::AutotunedSearch::nearestKSearch] This function is not supported by AutotunedSearch\n");
          return (0);
        }

        virtual int 
        radiusSearch (const PointT& point, const double radius, std::vector<int>& k_indices,    std::vector<float>& k_distances, int max_nn = -1) const;

        virtual int
        radiusSearch (const PointCloud& cloud, int index, double radius,
                      std::vector<int>& k_indices, std::vector<float>& k_distances,
                      int max_nn = -1);

        virtual int
        radiusSearch (int index, double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_distances, int max_nn = -1) const;


        virtual void
        approxNearestSearch (const PointCloudConstPtr &cloud_arg, int query_index_arg, int &result_index_arg,
                             float &sqr_distance_arg);

        virtual void
        approxNearestSearch (const PointT &p_q_arg, int &result_index_arg, float &sqr_distance_arg);

        virtual void
        approxNearestSearch (int query_index_arg, int &result_index_arg, float &sqr_distance_arg);

        inline int
        radiusSearch (std::vector<PointT, Eigen::aligned_allocator<PointT> > &point, 
                      std::vector <double> &radiuses, 
                      std::vector<std::vector<int> > &k_indices,
                      std::vector<std::vector<float> > &k_distances, 
                      int max_nn) const
        {
          PCL_ERROR("[pcl::search::AutotunedSearch::radiusSearch] This function is not supported by AutotunedSearch\n");
          return (0);
        }


        inline int
        approxRadiusSearch (const PointCloudConstPtr &cloud, 
                            int index, 
                            double radius,
                            std::vector<int> &k_indices, 
                            std::vector<float> &k_distances,
                            int max_nn) const
        {
          PCL_ERROR("[pcl::search::AutotunedSearch::approxRadiusSearch] This function is not supported by AutotunedSearch\n");
          return (0);
        }

        inline int
        approxNearestKSearch (const PointCloudConstPtr &cloud, 
                              int index, 
                              int k, 
                              std::vector<int> &k_indices, 
                              std::vector<float> &k_sqr_distances)
        {
          PCL_ERROR("[pcl::search::AutotunedSearch::approxNearestKSearch] This function is not supported by AutotunedSearch\n");
          return (0);
        }

      private:

        SearchPtr search_;

        int spatial_loc_;

        PointCloudConstPtr input_;
    };
  }
}

#endif  //#ifndef _PCL_SEARCH_AUTO_TUNED_SEARCH_H_
