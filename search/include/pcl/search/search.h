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

#ifndef PCL_SEARCH_SEARCH_H_
#define PCL_SEARCH_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>

namespace pcl
{
  namespace search
  {
    /** \brief Generic search class. All search wrappers must inherit from this. */
    template<typename PointT>
    class Search
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<pcl::search::Search<PointT> > Ptr;
        typedef boost::shared_ptr<const pcl::search::Search<PointT> > ConstPtr;

        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        /** Constructor. */
        Search ()
        {
        }

        /** Destructor. */
        virtual
        ~Search ()
        {
        }

        /** \brief Pass the input dataset that the search will be performed on.
          * \param[in] cloud a const pointer to the PointCloud data
          * \param[in] indices the point indices subset that is to be used from the cloud
          */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices) = 0;

        /** \brief Pass the input dataset that the search will be performed on.
          * \param[in] cloud a const pointer to the PointCloud data
          */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud) = 0;

        /** \brief Get a pointer to the input point cloud dataset. */
        virtual PointCloudConstPtr
        getInputCloud () = 0;

        /** \brief Get a pointer to the vector of indices used. */
        virtual IndicesConstPtr const
        getIndices () = 0;

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        virtual int
        nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances) = 0;

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        template <typename PointTDiff> int
        nearestKSearchT (const PointTDiff &point, int k, std::vector<int> &k_indices,
                         std::vector<float> &k_sqr_distances)
        {
          PointT p;
          // Copy all the data fields from the input cloud to the output one
          typedef typename pcl::traits::fieldList<PointT>::type FieldListInT;
          typedef typename pcl::traits::fieldList<PointTDiff>::type FieldListOutT;
          typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;
          pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointT, PointTDiff> (
                point, p));
          return (nearestKSearch (p, k, k_indices, k_sqr_distances));
        }

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        virtual int
        nearestKSearch (const PointCloud& cloud, int index, int k, std::vector<int>& k_indices,
                        std::vector<float>& k_sqr_distances) = 0;

        /** \brief Search for the k-nearest neighbors for the given query point (zero-copy).
         * \param[in] index the index representing the query point in the dataset given by \a setInputCloud
         *        if indices were given in setInputCloud, index will be the position in the indices vector
         * \param[in] k the number of neighbors to search for
         * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
         * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
         * a priori!)
         * \return number of neighbors found
         */
        virtual int
        nearestKSearch (int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances) = 0;

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] point the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        virtual int
        radiusSearch (const PointT& point, const double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_distances, int max_nn = -1) const = 0;

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] point the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        template <typename PointTDiff> int
        radiusSearchT (const PointTDiff& point, double radius, std::vector<int>& k_indices, 
                       std::vector<float>& k_distances, int max_nn = -1)
        {
          PointT p;
          // Copy all the data fields from the input cloud to the output one
          typedef typename pcl::traits::fieldList<PointT>::type FieldListInT;
          typedef typename pcl::traits::fieldList<PointTDiff>::type FieldListOutT;
          typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;
          pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointT, PointTDiff> (
                point, p));
          return (radiusSearch (p, radius, k_indices, k_distances, max_nn));
        }

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        virtual int
        radiusSearch (const PointCloud& cloud, int index, double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_distances, int max_nn = -1) = 0;

        /** \brief search for all the nearest neighbors of the query point in a given radius (zero-copy).
          * \param[in] index the index representing the query point in the dataset given by \a setInputCloud
          *        if indices were given in setInputCloud, index will be the position in the indices vector
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        virtual int
        radiusSearch (int index, double radius, std::vector<int>& k_indices, std::vector<float>& k_distances,
                      int max_nn = -1) const = 0;
    };
  }
}

#endif  //#ifndef _PCL_SEARCH_GENERIC_SEARCH_H_
