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
#include "pcl/search/pcl_search.h"

namespace pcl
{
  namespace search
  {
    enum SearchType
    {
      KDTREE, ORGANIZED_INDEX, OCTREE, AUTO_TUNED, NEAREST_K_SEARCH, NEAREST_RADIUS_SEARCH
    };

    /** \brief @b search::AutotunedSearch is a wrapper class which inherits all the search functions written in PCL
     *          and provides an intutive interface to all the functions. 
     */

    template<typename PointT>
      class AutotunedSearch : public pcl::search::Search<PointT>
      {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        typedef boost::shared_ptr<pcl::search::Search<PointT> > SearchPtr;
        typedef boost::shared_ptr<const pcl::search::Search<PointT> > SearchConstPtr;

        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        /** \brief constructor for AutotunedSearch. */
        AutotunedSearch (int spatial_locator) :
          search_ ()
        {
          initSearchDS (spatial_locator);
        }
        /* Constructor */
        AutotunedSearch () :
          search_ ()
        {
        }

        /*  Destructor */
        virtual
        ~AutotunedSearch ()
        {
        }

        /** \brief Initiate the search DS to point to spatial_locator
         * \param spatial_locator the spatial locator KDTREE, ORGANIZED_INDEX, OCTREE or AUTO_TUNED
         */
        void
        initSearchDS (int spatial_locator);

        /** \brief Evaluate the search Methods for the given cloud.
         * \param cloud the const boost shared pointer to a PointCloud message
         * \param search_type the search type NEAREST_K_SEARCH and NEAREST_RADIUS_SEARCH
         */
        void
        evaluateSearchMethods (const PointCloudConstPtr& cloud, const int search_type);

        /** \brief Provide a pointer to the input dataset.
         * \param cloud the const boost shared pointer to a PointCloud message
         * \param indices the point indices subset that is to be used from \a cloud - if NULL the whole point cloud is used
         */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices);

	virtual PointCloudConstPtr
        getInputCloud ()
	{
	   return (input_);
	}

	virtual IndicesConstPtr const
        getIndices ()
	{
	   return (indices_);
	}

        /** \brief Provide a pointer to the input dataset.
         * \param cloud the const boost shared pointer to a PointCloud message
         */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud);

        /** \brief search for k-nearest neighbors for the given query point.
         * \param point the given query point
         * \param k the number of neighbors to search for
         * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
         * \param k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
         * a priori!)
         * \return number of neighbors found
         */
        int
        nearestKSearch (const PointT& point, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances);

        /** \brief search for k-nearest neighbors for the given query point.
         * \param cloud the point cloud data
         * \param index the index in \a cloud representing the query point
         * \param k the number of neighbors to search for
         * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
         * \param k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
         * a priori!)
         * \return number of neighbors found
         */
        virtual int
        nearestKSearch (const PointCloud& cloud, int index, int k, std::vector<int>& k_indices,
                        std::vector<float>& k_sqr_distances);

        /** \brief search for k-nearest neighbors for the given query point (zero-copy).
         * \param index the index representing the query point in the dataset given by \a setInputCloud
         *        if indices were given in setInputCloud, index will be the position in the indices vector
         * \param k the number of neighbors to search for
         * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
         * \param k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
         * a priori!)
         * \return number of neighbors found
         */
        virtual int
        nearestKSearch (int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances);

        /** \brief search for k-nearest neighbors for the given query points.
         * \param point the given query points
         * \param k the numbers of the query point's neighbors to search for
         * \param k_indices the resultant indices of the neighboring points
         * \param k_sqr_distances the resultant squared distances to the neighboring points
         * \return number of neighbors found
         */
        inline int
        nearestKSearch (std::vector<PointT, Eigen::aligned_allocator<PointT> >& point, std::vector<int>& k,
                        std::vector<std::vector<int> >& k_indices, std::vector<std::vector<float> >& k_sqr_distances)
        {
          PCL_ERROR("[pcl::search::AutotunedSearch::nearestKSearch] This function is not supported by AutotunedSearch\n");
          return (0);
        }

        /** \brief search for all the nearest neighbors of the query point in a given radius.
         * \param point the given query point
         * \param radius the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        virtual int
        radiusSearch (const PointT& point, const double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_distances, int max_nn = -1) const;

        /** \brief search for all the nearest neighbors of the query point in a given radius.
         * \param cloud the point cloud data
         * \param index the index in \a cloud representing the query point
         * \param radius the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */

        virtual int
        radiusSearch (const PointCloud& cloud, int index, double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_distances, int max_nn = -1);

        /** \brief search for all the nearest neighbors of the query point in a given radius (zero-copy).
         * \param index the index representing the query point in the dataset given by \a setInputCloud
         *        if indices were given in setInputCloud, index will be the position in the indices vector
         * \param radius the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        virtual int
        radiusSearch (int index, double radius, std::vector<int>& k_indices, std::vector<float>& k_distances,
                      int max_nn = -1) const;

        /** \brief search for approximate nearest neighbor at the query point.
         * \param cloud_arg the const boost shared pointer to a PointCloud message
         * \param query_index_arg the index in \a cloud representing the query point
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         * \return number of neighbors found
         */
        virtual void
        approxNearestSearch (const PointCloudConstPtr &cloud_arg, int query_index_arg, int &result_index_arg,
                             float &sqr_distance_arg);

        /** \brief search for approximate nearest neighbor at the query point.
         * @param p_q_arg the given query point
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         */
        virtual void
        approxNearestSearch (const PointT &p_q_arg, int &result_index_arg, float &sqr_distance_arg);

        /** \brief search for approximate nearest neighbor at the query point.
         * \param query_index_arg index representing the query point in the dataset given by \a setInputCloud.
         *        If indices were given in setInputCloud, index will be the position in the indices vector.
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         */
        virtual void
        approxNearestSearch (int query_index_arg, int &result_index_arg, float &sqr_distance_arg);

        /** \brief Approximate search for all the nearest neighbors of the query points in the given radiuses.
         * \param point the given query points
         * \param radiuses the radiuses of the sphere bounding all of point's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radiuses
         */
        inline int
        radiusSearch (std::vector<PointT, Eigen::aligned_allocator<PointT> > &point, std::vector<double> &radiuses,
                      std::vector<std::vector<int> > &k_indices, std::vector<std::vector<float> > &k_distances,
                      int max_nn) const
        {
          PCL_ERROR("[pcl::search::AutotunedSearch::radiusSearch] This function is not supported by AutotunedSearch\n");
          return (0);
        }

        /** \brief Approximate search for all the nearest neighbors of the query point in a given radius.
         * \param cloud the const boost shared pointer to a PointCloud message
         * \param index the index in \a cloud representing the query point
         * \param radius the radius of the sphere bounding all of point's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        inline int
        approxRadiusSearch (const PointCloudConstPtr &cloud, int index, double radius, std::vector<int> &k_indices,
                            std::vector<float> &k_distances, int max_nn) const
        {
          PCL_ERROR("[pcl::search::AutotunedSearch::approxRadiusSearch] This function is not supported by AutotunedSearch\n");
          return (0);
        }
        /** \brief Approximate search for k-nearest neighbors for the given query point.
         * \param cloud the const boost shared pointer to a PointCloud message
         * \param index the index in \a cloud representing the query point
         * \param k the number of neighbors to search for
         * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
         * \param k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
         * a priori!)
         * \return number of neighbors found
         */
        inline int
        approxNearestKSearch (const PointCloudConstPtr &cloud, int index, int k, std::vector<int> &k_indices,
                              std::vector<float> &k_sqr_distances)
        {
          PCL_ERROR("[pcl::search::AutotunedSearch::approxNearestKSearch] This function is not supported by AutotunedSearch\n");
          return (0);
        }

      private:

        SearchPtr search_;

        int spatial_loc_;

        PointCloudConstPtr input_;
	IndicesConstPtr indices_;
      };
  }
}

#endif  //#ifndef _PCL_SEARCH_AUTO_TUNED_SEARCH_H_
