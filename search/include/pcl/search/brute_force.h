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
 */

#ifndef PCL_SEARCH_BRUTE_FORCE_H_
#define PCL_SEARCH_BRUTE_FORCE_H_

#include <pcl/search/search.h>

namespace pcl
{
  namespace search
  {
    /** \brief Implementation of the brute force search algorithm
      * \author Suat Gedikli <gedikli@willowgarage.com>
      * \ingroup search
      */
    template<typename PointT>
    class BruteForce: public Search<PointT>
    {
      typedef typename Search<PointT>::PointCloud PointCloud;
      typedef typename Search<PointT>::PointCloudConstPtr PointCloudConstPtr;

      typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
      typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

      struct Entry
      {
        Entry (int idx, float dist) : index (idx), distance (dist) {}
        Entry () {}
        unsigned index;
        float distance;
        bool operator < (const Entry& other) const
        {
          return distance < other.distance;
        }
      };

      // replace by some metric functor
      float getDistSqr (const PointT& point1, const PointT& point2) const;
      public:
        BruteForce ()
        {
        }

        /** \brief Destructor for KdTree. */
        virtual
        ~BruteForce ()
        {
        }

        /** \brief Provide a pointer to the input dataset.
          * \param cloud the const boost shared pointer to a PointCloud message
          * \param indices the point indices subset that is to be used from \a cloud
          */
        inline void
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices)
        {
          cloud_ = cloud;
          indices_ = indices;
        }

        /** \brief Provide a pointer to the input dataset.
          * \param cloud the const boost shared pointer to a PointCloud message
          */
        inline void
        setInputCloud (const PointCloudConstPtr& cloud)
        {
          const IndicesConstPtr& indices = IndicesConstPtr ();
          setInputCloud (cloud, indices);
        }

        /** \brief Get a pointer to the input dataset as passed by the user. */
        PointCloudConstPtr
        getInputCloud ()
        {
          return (cloud_);
        }

        /** \brief Get a pointer to the set of input indices used as passed by the user. */
        virtual IndicesConstPtr const
        getIndices ()
        {
          return (indices_);
        }

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        int
        nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);


        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        inline int
        nearestKSearch (const PointCloud &cloud, int index, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_distances)
        {
          return nearestKSearch (cloud[index], k, k_indices, k_distances);
        }

        /** \brief Search for the k-nearest neighbors for the given query point (zero-copy).
          *
          * \param[in] index the index representing the query point in the
          * dataset given by \a setInputCloud if indices were given in
          * setInputCloud, index will be the position in the indices vector
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        inline int
        nearestKSearch (int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
        {
          return nearestKSearch (cloud_->operator[](index), k, k_indices, k_distances);
        }

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] point the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        int
        radiusSearch (const PointT& point, double radius,
                      std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
                      int max_nn = -1) const;

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        inline int
        radiusSearch (const PointCloud& cloud, int index, double radius, std::vector<int> &k_indices,
                      std::vector<float> &k_distances, int max_nn = -1)
        {
          return (radiusSearch (cloud[index], radius, k_indices, k_distances, max_nn));
        }

        /** \brief Search for all the nearest neighbors of the query point in a given radius (zero-copy).
          * \param[in] index the index representing the query point in the dataset given by \a setInputCloud
          *        if indices were given in setInputCloud, index will be the position in the indices vector
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        inline int
        radiusSearch (int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances,
                      int max_nn = -1) const
        {
          return (radiusSearch (cloud_->operator[](index), radius, k_indices, k_distances, max_nn));
        }

      protected:
        PointCloudConstPtr cloud_;
        IndicesConstPtr indices_;
    };
  }
}

#endif    // PCL_SEARCH_BRUTE_FORCE_H_
