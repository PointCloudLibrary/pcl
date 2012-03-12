/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_FILTERS_RADIUSOUTLIERREMOVAL_H_
#define PCL_FILTERS_RADIUSOUTLIERREMOVAL_H_

#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/search/pcl_search.h>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b RadiusOutlierRemoval is a simple filter that removes outliers if the number of neighbors in a certain
    * search radius is smaller than a given K.
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<typename PointT>
  class RadiusOutlierRemoval : public Filter<PointT>
  {
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;

    using Filter<PointT>::removed_indices_;
    using Filter<PointT>::extract_removed_indices_;

    typedef typename pcl::search::Search<PointT> KdTree;
    typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      RadiusOutlierRemoval (bool extract_removed_indices = false) :
        Filter<PointT>::Filter (extract_removed_indices), search_radius_ (0.0), min_pts_radius_ (1), tree_ ()
      {
        filter_name_ = "RadiusOutlierRemoval";
      }
      ;

      /** \brief Set the sphere radius that is to be used for determining the k-nearest neighbors for filtering.
        * \param radius the sphere radius that is to contain all k-nearest neighbors
        */
      inline void
      setRadiusSearch (double radius)
      {
        search_radius_ = radius;
      }

      /** \brief Get the sphere radius used for determining the k-nearest neighbors. */
      inline double
      getRadiusSearch ()
      {
        return (search_radius_);
      }

      /** \brief Set the minimum number of neighbors that a point needs to have in the given search radius in order to
        * be considered an inlier (i.e., valid).
        * \param min_pts the minimum number of neighbors
        */
      inline void
      setMinNeighborsInRadius (int min_pts)
      {
        min_pts_radius_ = min_pts;
      }

      /** \brief Get the minimum number of neighbors that a point needs to have in the given search radius to be
        * considered an inlier and avoid being filtered. 
        */
      inline double
      getMinNeighborsInRadius ()
      {
        return (min_pts_radius_);
      }

    protected:
      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief The minimum number of neighbors that a point needs to have in the given search radius to be considered
        * an inlier. 
        */
      int min_pts_radius_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief Apply the filter
        * \param output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b RadiusOutlierRemoval is a simple filter that removes outliers if the number of neighbors in a certain
    * search radius is smaller than a given K.
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS RadiusOutlierRemoval<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    using Filter<sensor_msgs::PointCloud2>::filter_name_;
    using Filter<sensor_msgs::PointCloud2>::getClassName;

    using Filter<sensor_msgs::PointCloud2>::removed_indices_;
    using Filter<sensor_msgs::PointCloud2>::extract_removed_indices_;

    typedef pcl::search::Search<pcl::PointXYZ> KdTree;
    typedef pcl::search::Search<pcl::PointXYZ>::Ptr KdTreePtr;

    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
      /** \brief Empty constructor. */
      RadiusOutlierRemoval (bool extract_removed_indices = false) :
        Filter<sensor_msgs::PointCloud2>::Filter (extract_removed_indices), 
        search_radius_ (0.0), min_pts_radius_ (1), tree_ ()
      {
        filter_name_ = "RadiusOutlierRemoval";
      }

      /** \brief Set the sphere radius that is to be used for determining the k-nearest neighbors for filtering.
        * \param radius the sphere radius that is to contain all k-nearest neighbors
        */
      inline void
      setRadiusSearch (double radius)
      {
        search_radius_ = radius;
      }

      /** \brief Get the sphere radius used for determining the k-nearest neighbors. */
      inline double
      getRadiusSearch ()
      {
        return (search_radius_);
      }

      /** \brief Set the minimum number of neighbors that a point needs to have in the given search radius in order to
        * be considered an inlier (i.e., valid).
        * \param min_pts the minimum number of neighbors
        */
      inline void
      setMinNeighborsInRadius (int min_pts)
      {
        min_pts_radius_ = min_pts;
      }

      /** \brief Get the minimum number of neighbors that a point needs to have in the given search radius to be
        * considered an inlier and avoid being filtered. 
        */
      inline double
      getMinNeighborsInRadius ()
      {
        return (min_pts_radius_);
      }

    protected:
      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief The minimum number of neighbors that a point needs to have in the given search radius to be considered
        * an inlier. 
        */
      int min_pts_radius_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      void
      applyFilter (PointCloud2 &output);
  };
}

#endif  //#ifndef PCL_FILTERS_RADIUSOUTLIERREMOVAL_H_
