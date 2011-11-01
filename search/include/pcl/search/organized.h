/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_SEARCH_ORGANIZED_NEIGHBOR_SEARCH_H_
#define PCL_SEARCH_ORGANIZED_NEIGHBOR_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

#include <algorithm>
#include <math.h>
#include <queue>
#include <vector>

namespace pcl
{
  namespace search
  {
    /** \brief @b OrgfanizedNeighbor is a class for optimized nearest neigbhor search in organized point clouds.
      *
      * \author Radu B. Rusu, Julius Kammerl, Suat Gedikli
      */
    template<typename PointT>
    class OrganizedNeighbor : public pcl::search::Search<PointT>
    {
      // public typedefs
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef boost::shared_ptr<PointCloud> PointCloudPtr;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
      typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

      public:

        /** \brief OrganizedNeighbor constructor. */
        OrganizedNeighbor ()
        {
          max_distance_ = std::numeric_limits<double>::max ();
          horizontal_window_ = 0;
          vertical_window_ = 0;
        }

        /** \brief Empty deconstructor. */
        ~OrganizedNeighbor ()
        {
        }

        /** \brief Provide a pointer to the input data set.
         *  \param cloud the const boost shared pointer to a PointCloud message
         */
        inline void
        setInputCloud (const PointCloudConstPtr &cloud)
        {
          if (input_ != cloud)
            input_ = cloud;
        }

        /** \brief Get a pointer to the input dataset as passed by the user. */
        PointCloudConstPtr
        getInputCloud ()
        {
          return input_;
        }

        /** \brief Get the maximum allowed distance between the query point and its nearest neighbors. */
        inline double
        getMaxDistance () const
        {
          return (max_distance_);
        }

        /** \brief Set the maximum allowed distance between the query point and its nearest neighbors. */
        inline void
        setMaxDistance (double max_dist)
        {
          max_distance_ = max_dist;
        }

        /** \brief set the search window (horizontal, vertical) in pixels.
         * \param horizontal the horizontal window in pixel
         * \param vertical the vertical window in pixel
         */
        inline void
        setSearchWindow (int horizontal, int vertical)
        {
          horizontal_window_ = horizontal;
          vertical_window_ = vertical;
        }

        /** \brief Estimate the search window (horizontal, vertical) in pixels in order to get up to k-neighbors.
         * \param k the number of neighbors requested
         */
        void
        setSearchWindowAsK (int k);

        /** \brief Get the horizontal search window in pixels. */
        int
        getHorizontalSearchWindow () const
        {
          return (horizontal_window_);
        }

        /** \brief Get the vertical search window in pixels. */
        int
        getVerticalSearchWindow () const
        {
          return (vertical_window_);
        }

        /** \brief Search for the k-nearest neighbors for a given query point.
          * \note limiting the maximum search radius (with setMaxDistance) can lead to a significant improvement in search speed
          * \param[in] p_q the given query point (\ref setInputCloud must be given a-priori!)
          * \param[in] k the number of neighbors to search for (used only if \ref horizontal and vertical window not given already!)
          * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
          * \param[out] k_distances \note this function does not return distances
          * \return number of neighbors found
          */
        int
        nearestKSearch (const PointT &p_q, int k, std::vector<int> &k_indices,
                              std::vector<float> &k_sqr_distances)
        {
          PCL_ERROR ("[pcl::search::OrganizedNeighbor::approxNearestKSearch] Method not implemented!\n");
          return (0);
        }

        /** \brief Search for the k-nearest neighbors for the given query point (zero-copy).
          * \note limiting the maximum search radius (with setMaxDistance) can lead to a significant improvement in search speed
          *
          * \param[in] index the index representing the query point in the dataset (\ref setInputCloud must be given a-priori!)
          * \param[in] k the number of neighbors to search for (used only if \ref horizontal and vertical window not given already!)
          * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
          * \param[out] k_distances \note this function does not return distances
          * \return number of neighbors found
          */
        int
        nearestKSearch (int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);

        /** \brief Search for the k-nearest neighbors for a given query point.
          * \note limiting the maximum search radius (with setMaxDistance) can lead to a significant improvement in search speed
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] k the number of neighbors to search for (used only if \ref horizontal and vertical window not given already!)
          * \param[out] k_indices the resultant point indices (must be resized to \a k beforehand!)
          * \param[out] k_distances \note this function does not return distances
          * \return number of neighbors found
          */
        int
        nearestKSearch (const pcl::PointCloud<PointT> &cloud, int index, int k, 
                              std::vector<int> &k_indices, std::vector<float> &k_distances);

        /** \brief Approximate search for neighbors around the given query point within radius.
          * \param[in] cloud the point cloud data.
          * \param[in] index the index in \a cloud representing the query point.
          * \param[in] radius the maximum distance to search for neighbors in.
          * \param[out] k_indices the resultant point indices
          * \param[out] k_distances the resultant !squared! point distances
          * \param[in] max_nn maximum number of points to return
          */
        int
        radiusSearch (const pcl::PointCloud<PointT> &cloud, int index, double radius, 
                            std::vector<int> &k_indices, std::vector<float> &k_distances, 
                            int max_nn = INT_MAX);

        /** \brief Approximate search for neighbors around the given query point within radius.
          * \param[in] index the index representing the query point in the dataset (\ref setInputCloud must be given a-priori!)
          * \param[in] radius the maximum distance to search for neighbors in.
          * \param[out] k_indices the resultant point indices
          * \param[out] k_distances the resultant !squared! point distances
          * \param[in] max_nn maximum number of points to return
          */
        int
        radiusSearch (int index, double radius, 
                            std::vector<int> &k_indices, std::vector<float> &k_distances, 
                            int max_nn = INT_MAX) const;

        /** \brief Approximate search for neighbors around the given query point within radius.
          * \param[in] point the given query point
          * \param[in] radius the maximum distance to search for neighbors in.
          * \param[out] k_indices the resultant point indices
          * \param[out] k_distances the resultant !squared! point distances
          * \param[in] max_nn maximum number of points to return
          */
        int
        radiusSearch (const PointT &p_q, double radius, 
                            std::vector<int> &k_indices, std::vector<float> &k_distances, 
                            int max_nn = INT_MAX) const
        {
          PCL_ERROR ("[pcl::search::OrganizedNeighbor::radiusSearch] Method not implemented!\n");
          return (0);
        }

      protected:
        /** \brief Class getName method. */
        virtual std::string
        getName () const
        {
          return ("Organized_Neighbor_Search");
        }

        /** \brief The horizontal search window. */
        int horizontal_window_;int vertical_window_;int min_pts_;

        /** \brief Pointer to input point cloud dataset. */
        PointCloudConstPtr input_;

        /** \brief Maximum allowed distance between the query point and its k-neighbors. */
        double max_distance_;
    };
  }
}

#endif

