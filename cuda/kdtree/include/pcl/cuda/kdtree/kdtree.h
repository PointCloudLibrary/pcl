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
 * kdtree.h
 * Adapted from: kdtree/include/pcl/kdtree/kdtree.h
 * Created on: Jun 01, 2022
 * Author: Ramzi Sabra
 */

#pragma once

#include <pcl/kdtree/base.h>

namespace pcl {
namespace cuda {
  /** \brief KdTree represents the base spatial locator class for kd-tree CUDA implementations.
    * \author Ramzi Sabra
    * \ingroup cuda/kdtree
    */
  template <typename PointT>
  class KdTree : public KdTreeBase<PointT>
  {
    using Base = KdTreeBase<PointT>;

    public:
      using PointVector = std::vector<PointT, Eigen::aligned_allocator<PointT>>;
      using IndexMatrix = Eigen::Matrix<index_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
      using DistanceMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

      using IndicesVector = std::vector<Indices>;
      using DistancesVector = std::vector<std::vector<float>>;

      // Boost shared pointers
      using Ptr = shared_ptr<KdTree<PointT> >;
      using ConstPtr = shared_ptr<const KdTree<PointT> >;

      /** \brief Empty constructor for KdTree. Sets some internal values to their defaults.
        * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise.
        * \param[in] max_leaf_size maximum leaf node size. Set to 64 by default.
        */
      KdTree (bool sorted = true, int max_leaf_size = 64)
        : Base(sorted, max_leaf_size)
      {
      };

      /** \brief Search for k-nearest neighbors for the given query points.
        * \param[in] points the given query points vector
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices_mat the resultant indices matrix of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances matrix to the neighboring points
        * \return number of neighbors found
        */
      virtual int
      nearestKSearch (const PointVector& points,
                      unsigned int k,
                      IndexMatrix& k_indices_mat,
                      DistanceMatrix& k_sqr_distances_mat) const = 0;

      /** \brief Search for k-nearest neighbors for the given query points.
        * \param[in] points the given query points
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices_mat the resultant indices vector of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances vector to the neighboring points
        * \return number of neighbors found
        */
      virtual int
      nearestKSearch (const PointVector& points,
                      unsigned int k,
                      IndicesVector& k_indices_vec,
                      DistancesVector& k_sqr_distances_vec) const = 0;

      /** \brief Search for all the nearest neighbors of the query points in a given radius.
        * \param[in] points the given query points vector
        * \param[in] radius the radius of the spheres bounding all of each query point's neighbors
        * \param[out] k_indices the resultant indices matrix of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances matrix to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
        * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
        * returned.
        * \return number of neighbors found in radius
        */
      virtual int
      radiusSearch (const PointVector& points,
                    double radius,
                    IndexMatrix& indices_mat,
                    DistanceMatrix& sqr_distances_mat,
                    unsigned int max_nn = 0) const = 0;
      
      /** \brief Search for all the nearest neighbors of the query points in a given radius.
        * \param[in] points the given query points vector
        * \param[in] radius the radius of the spheres bounding all of each query point's neighbors
        * \param[out] k_indices the resultant indices vector of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances vector to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
        * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
        * returned.
        * \return number of neighbors found in radius
        */
      virtual int
      radiusSearch (const PointVector& points,
                    double radius,
                    IndicesVector& indices_vec,
                    DistancesVector& sqr_distances_vec,
                    unsigned int max_nn = 0) const = 0;

    protected:
      using Base::input_;
      using Base::indices_;
  };
} // namespace cuda
} // namespace pcl
