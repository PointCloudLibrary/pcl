/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/common/copy_point.h>

namespace pcl
{
  template <typename PointT>
  class KdTreeBase
  {
    public:
      using IndicesPtr = shared_ptr<Indices >;
      using IndicesConstPtr = shared_ptr<const Indices >;

      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      using PointRepresentation = pcl::PointRepresentation<PointT>;
      using PointRepresentationConstPtr = typename PointRepresentation::ConstPtr;

      /** \brief Empty constructor for KdTreeBase. Sets some internal values to their defaults.
        * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise.
        * \param[in] max_leaf_size maximum leaf node size.
        */
      KdTreeBase (bool sorted, int max_leaf_size)
        : input_()
        , max_leaf_size_(max_leaf_size)
        , epsilon_(0.0f)
        , min_pts_(1)
        , sorted_(sorted)
        , point_representation_ (new DefaultPointRepresentation<PointT>)
      {
      }

      /** \brief Destructor for KdTreeBase. Deletes all allocated data arrays and destroys the kd-tree structures. */
      virtual ~KdTreeBase () {};

      /** \brief Provide a pointer to the input dataset.
        * \param[in] cloud the const shared pointer to a PointCloud message
        * \param[in] indices the point indices subset that is to be used from \a cloud - if NULL the whole cloud is used
        */
      virtual void
      setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr ())
      {
        input_   = cloud;
        indices_ = indices;
      }

      /** \brief Get a pointer to the input point cloud dataset. */
      inline PointCloudConstPtr
      getInputCloud () const
      {
        return (input_);
      }

      /** \brief Get a pointer to the vector of indices used. */
      inline IndicesConstPtr
      getIndices () const
      {
        return (indices_);
      }

      /** \brief Set the maximum leaf node size.
       * \param[in] max_leaf_size maximum leaf node size
       */
      virtual inline void
      setMaxLeafSize(int max_leaf_size)
      {
        max_leaf_size_ = max_leaf_size;
      }

      /** \brief Get the maximum leaf node size. */
      inline int getMaxLeafSize() const
      {
        return (max_leaf_size_);
      }
      
      /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
        * \param[in] eps precision (error bound) for nearest neighbors searches
        */
      virtual inline void
      setEpsilon (float eps)
      {
        epsilon_ = eps;
      }

      /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
      inline float
      getEpsilon () const
      {
        return (epsilon_);
      }

      /** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain.
        * \param[in] min_pts the minimum number of neighbors in a viable neighborhood
        */
      inline void
      setMinPts (int min_pts)
      {
        min_pts_ = min_pts;
      }

      /** \brief Get the minimum allowed number of k nearest neighbors points that a viable result must contain. */
      inline int
      getMinPts () const
      {
        return (min_pts_);
      }

      /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors.
        * \param[in] point_representation the const shared pointer to a PointRepresentation
        */
      virtual inline void
      setPointRepresentation (const PointRepresentationConstPtr &point_representation)
      {
        point_representation_ = point_representation;
        if (!input_) return;
        setInputCloud (input_, indices_);  // Makes sense in derived classes to reinitialize the tree
      }

      /** \brief Get a pointer to the point representation used when converting points into k-D vectors. */
      inline PointRepresentationConstPtr
      getPointRepresentation () const
      {
        return (point_representation_);
      }

    protected:
      /** \brief The input point cloud dataset containing the points we need to use. */
      PointCloudConstPtr input_;

      /** \brief A pointer to the vector of point indices to use. */
      IndicesConstPtr indices_;

      /** \brief Maximum leaf node size */
      int max_leaf_size_;
      
      /** \brief Epsilon precision (error bound) for nearest neighbors searches. */
      float epsilon_;

      /** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain. */
      int min_pts_;

      /** \brief Return the radius search neighbours sorted **/
      bool sorted_;

      /** \brief For converting different point structures into k-dimensional vectors for nearest-neighbor search. */
      PointRepresentationConstPtr point_representation_;

      /** \brief Class getName method. */
      virtual std::string
      getName () const = 0;
  };
}
