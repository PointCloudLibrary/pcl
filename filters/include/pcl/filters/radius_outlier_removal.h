/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#pragma once

#include <pcl/filters/filter_indices.h>
#include <pcl/search/search.h> // for Search, Search<>::Ptr

namespace pcl
{
  /** \brief @b RadiusOutlierRemoval filters points in a cloud based on the number of neighbors they have.
    * \details Iterates through the entire input once, and for each point, retrieves the number of neighbors within a certain radius.
    * The point will be considered an outlier if it has too few neighbors, as determined by setMinNeighborsInRadius().
    * The radius can be changed using setRadiusSearch().
    * <br>
    * The neighbors found for each query point will be found amongst ALL points of setInputCloud(), not just those indexed by setIndices().
    * The setIndices() method only indexes the points that will be iterated through as search query points.
    * <br><br>
    * Usage example:
    * \code
    * pcl::RadiusOutlierRemoval<PointType> rorfilter (true); // Initializing with true will allow us to extract the removed indices
    * rorfilter.setInputCloud (cloud_in);
    * rorfilter.setRadiusSearch (0.1);
    * rorfilter.setMinNeighborsInRadius (5);
    * rorfilter.setNegative (true);
    * rorfilter.filter (*cloud_out);
    * // The resulting cloud_out contains all points of cloud_in that have 4 or less neighbors within the 0.1 search radius
    * indices_rem = rorfilter.getRemovedIndices ();
    * // The indices_rem array indexes all points of cloud_in that have 5 or more neighbors within the 0.1 search radius
    * \endcode
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<typename PointT>
  class RadiusOutlierRemoval : public FilterIndices<PointT>
  {
    protected:
      using PointCloud = typename FilterIndices<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;
      using SearcherPtr = typename pcl::search::Search<PointT>::Ptr;

    public:

      using Ptr = shared_ptr<RadiusOutlierRemoval<PointT> >;
      using ConstPtr = shared_ptr<const RadiusOutlierRemoval<PointT> >;


      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
      RadiusOutlierRemoval (bool extract_removed_indices = false) :
        FilterIndices<PointT> (extract_removed_indices),
        searcher_ (),
        search_radius_ (0.0),
        min_pts_radius_ (1)
      {
        filter_name_ = "RadiusOutlierRemoval";
      }

      /** \brief Set the radius of the sphere that will determine which points are neighbors.
        * \details The number of points within this distance from the query point will need to be equal or greater
        * than setMinNeighborsInRadius() in order to be classified as an inlier point (i.e. will not be filtered).
        * \param[in] radius The radius of the sphere for nearest neighbor searching.
        */
      inline void
      setRadiusSearch (double radius)
      {
        search_radius_ = radius;
      }

      /** \brief Get the radius of the sphere that will determine which points are neighbors.
        * \details The number of points within this distance from the query point will need to be equal or greater
        * than setMinNeighborsInRadius() in order to be classified as an inlier point (i.e. will not be filtered).
        * \return The radius of the sphere for nearest neighbor searching.
        */
      inline double
      getRadiusSearch ()
      {
        return (search_radius_);
      }

      /** \brief Set the number of neighbors that need to be present in order to be classified as an inlier.
        * \details The number of points within setRadiusSearch() from the query point will need to be equal or greater
        * than this number in order to be classified as an inlier point (i.e. will not be filtered).
        * \param min_pts The minimum number of neighbors (default = 1).
        */
      inline void
      setMinNeighborsInRadius (int min_pts)
      {
        min_pts_radius_ = min_pts;
      }

      /** \brief Get the number of neighbors that need to be present in order to be classified as an inlier.
        * \details The number of points within setRadiusSearch() from the query point will need to be equal or greater
        * than this number in order to be classified as an inlier point (i.e. will not be filtered).
        * \return The minimum number of neighbors (default = 1).
        */
      inline int
      getMinNeighborsInRadius ()
      {
        return (min_pts_radius_);
      }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilter (Indices &indices) override
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilterIndices (Indices &indices);

    private:
      /** \brief A pointer to the spatial search object. */
      SearcherPtr searcher_;

      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief The minimum number of neighbors that a point needs to have in the given search radius to be considered an inlier. */
      int min_pts_radius_;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b RadiusOutlierRemoval is a simple filter that removes outliers if the number of neighbors in a certain
    * search radius is smaller than a given K.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS RadiusOutlierRemoval<pcl::PCLPointCloud2> : public FilterIndices<pcl::PCLPointCloud2>
  {
    using Filter<pcl::PCLPointCloud2>::filter_name_;
    using Filter<pcl::PCLPointCloud2>::getClassName;

    using Filter<pcl::PCLPointCloud2>::removed_indices_;
    using Filter<pcl::PCLPointCloud2>::extract_removed_indices_;

    using KdTree = pcl::search::Search<pcl::PointXYZ>;
    using KdTreePtr = pcl::search::Search<pcl::PointXYZ>::Ptr;

    using PCLPointCloud2 = pcl::PCLPointCloud2;
    using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
    using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

    public:
      /** \brief Empty constructor. */
      RadiusOutlierRemoval (bool extract_removed_indices = false) :
        FilterIndices<pcl::PCLPointCloud2>::FilterIndices (extract_removed_indices),
        search_radius_ (0.0), min_pts_radius_ (1)
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
      KdTreePtr searcher_;

      void
      applyFilter (PCLPointCloud2 &output) override;

      void
      applyFilter (Indices &indices) override;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#endif
