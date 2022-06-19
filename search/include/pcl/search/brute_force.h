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
 */

#pragma once

#include <pcl/search/search.h>

namespace pcl
{
  namespace search
  {
    /** \brief Implementation of a simple brute force search algorithm.
      * \author Suat Gedikli
      * \ingroup search
      */
    template<typename PointT>
    class BruteForce: public Search<PointT>
    {
      using PointCloud = typename Search<PointT>::PointCloud;
      using PointCloudConstPtr = typename Search<PointT>::PointCloudConstPtr;

      using IndicesPtr = pcl::IndicesPtr;
      using IndicesConstPtr = pcl::IndicesConstPtr;

      using pcl::search::Search<PointT>::input_;
      using pcl::search::Search<PointT>::indices_;
      using pcl::search::Search<PointT>::sorted_results_;

      struct Entry
      {
        Entry (index_t idx, float dist) : index (idx), distance (dist) {}

        Entry () : index (0), distance (0) {}
        index_t index;
        float distance;
        
        inline bool 
        operator < (const Entry& other) const
        {
          return (distance < other.distance);
        }
        
        inline bool 
        operator > (const Entry& other) const
        {
          return (distance > other.distance);
        }
      };

      // replace by some metric functor
      float getDistSqr (const PointT& point1, const PointT& point2) const;
      public:
        BruteForce (bool sorted_results = false)
        : Search<PointT> ("BruteForce", sorted_results)
        {
        }

        /** \brief Destructor for KdTree. */
        
        ~BruteForce () override = default;

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        int
        nearestKSearch (const PointT &point, int k, Indices &k_indices, std::vector<float> &k_distances) const override;

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] point the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
          * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
          * returned.
          * \return number of neighbors found in radius
          */
        int
        radiusSearch (const PointT& point, double radius,
                      Indices &k_indices, std::vector<float> &k_sqr_distances,
                      unsigned int max_nn = 0) const override;

      private:
        int
        denseKSearch (const PointT &point, int k, Indices &k_indices, std::vector<float> &k_distances) const;

        int
        sparseKSearch (const PointT &point, int k, Indices &k_indices, std::vector<float> &k_distances) const;

        int
        denseRadiusSearch (const PointT& point, double radius,
                           Indices &k_indices, std::vector<float> &k_sqr_distances,
                           unsigned int max_nn = 0) const;

        int
        sparseRadiusSearch (const PointT& point, double radius,
                            Indices &k_indices, std::vector<float> &k_sqr_distances,
                            unsigned int max_nn = 0) const;
    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/search/impl/brute_force.hpp>
#endif
