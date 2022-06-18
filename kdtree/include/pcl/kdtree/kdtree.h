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
  /** \brief KdTree represents the base spatial locator class for kd-tree implementations.
    * \author Radu B Rusu, Bastian Steder, Michael Dixon
    * \ingroup kdtree
    */
  template <typename PointT>
  class KdTree
  {
    public:
      using IndicesPtr = shared_ptr<Indices >;
      using IndicesConstPtr = shared_ptr<const Indices >;

      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      using PointRepresentation = pcl::PointRepresentation<PointT>;
      using PointRepresentationConstPtr = typename PointRepresentation::ConstPtr;

      // Boost shared pointers
      using Ptr = shared_ptr<KdTree<PointT> >;
      using ConstPtr = shared_ptr<const KdTree<PointT> >;

      /** \brief Empty constructor for KdTree. Sets some internal values to their defaults.
        * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise.
        */
      KdTree (bool sorted = true) : input_(),
                                    epsilon_(0.0f), min_pts_(1), sorted_(sorted),
                                    point_representation_ (new DefaultPointRepresentation<PointT>)
      {
      };

      /** \brief Provide a pointer to the input dataset.
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        * \param[in] indices the point indices subset that is to be used from \a cloud - if NULL the whole cloud is used
        */
      virtual void
      setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr ())
      {
        input_   = cloud;
        indices_ = indices;
      }

      /** \brief Get a pointer to the vector of indices used. */
      inline IndicesConstPtr
      getIndices () const
      {
        return (indices_);
      }

      /** \brief Get a pointer to the input point cloud dataset. */
      inline PointCloudConstPtr
      getInputCloud () const
      {
        return (input_);
      }

      /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors.
        * \param[in] point_representation the const boost shared pointer to a PointRepresentation
        */
      inline void
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

      /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
      virtual ~KdTree () = default;

      /** \brief Search for k-nearest neighbors for the given query point.
        * \param[in] p_q the given query point
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        * \return number of neighbors found
        */
      virtual int
      nearestKSearch (const PointT &p_q, unsigned int k,
                      Indices &k_indices, std::vector<float> &k_sqr_distances) const = 0;

      /** \brief Search for k-nearest neighbors for the given query point.
        *
        * \attention This method does not do any bounds checking for the input index
        * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
        *
        * \param[in] cloud the point cloud data
        * \param[in] index a \a valid index in \a cloud representing a \a valid (i.e., finite) query point
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        *
        * \return number of neighbors found
        *
        * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
        */
      virtual int
      nearestKSearch (const PointCloud &cloud, int index, unsigned int k,
                      Indices &k_indices, std::vector<float> &k_sqr_distances) const
      {
        assert (index >= 0 && index < static_cast<int> (cloud.size ()) && "Out-of-bounds error in nearestKSearch!");
        return (nearestKSearch (cloud[index], k, k_indices, k_sqr_distances));
      }

      /** \brief Search for k-nearest neighbors for the given query point.
        * This method accepts a different template parameter for the point type.
        * \param[in] point the given query point
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        * \return number of neighbors found
        */
      template <typename PointTDiff> inline int
      nearestKSearchT (const PointTDiff &point, unsigned int k,
                       Indices &k_indices, std::vector<float> &k_sqr_distances) const
      {
        PointT p;
        copyPoint (point, p);
        return (nearestKSearch (p, k, k_indices, k_sqr_distances));
      }

      /** \brief Search for k-nearest neighbors for the given query point (zero-copy).
        *
        * \attention This method does not do any bounds checking for the input index
        * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
        *
        * \param[in] index a \a valid index representing a \a valid query point in the dataset given
        * by \a setInputCloud. If indices were given in setInputCloud, index will be the position in
        * the indices vector.
        *
        * \param[in] k the number of neighbors to search for
        * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        * \return number of neighbors found
        *
        * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
        */
      virtual int
      nearestKSearch (int index, unsigned int k,
                      Indices &k_indices, std::vector<float> &k_sqr_distances) const
      {
        if (indices_ == nullptr)
        {
          assert (index >= 0 && index < static_cast<int> (input_->size ()) && "Out-of-bounds error in nearestKSearch!");
          return (nearestKSearch ((*input_)[index], k, k_indices, k_sqr_distances));
        }
        assert (index >= 0 && index < static_cast<int> (indices_->size ()) && "Out-of-bounds error in nearestKSearch!");

        return (nearestKSearch ((*input_)[(*indices_)[index]], k, k_indices, k_sqr_distances));
      }

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
        * \param[in] p_q the given query point
        * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
        * \param[out] k_indices the resultant indices of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
        * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
        * returned.
        * \return number of neighbors found in radius
        */
      virtual int
      radiusSearch (const PointT &p_q, double radius, Indices &k_indices,
                    std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const = 0;

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
        *
        * \attention This method does not do any bounds checking for the input index
        * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
        *
        * \param[in] cloud the point cloud data
        * \param[in] index a \a valid index in \a cloud representing a \a valid (i.e., finite) query point
        * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
        * \param[out] k_indices the resultant indices of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
        * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
        * returned.
        * \return number of neighbors found in radius
        *
        * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
        */
      virtual int
      radiusSearch (const PointCloud &cloud, int index, double radius,
                    Indices &k_indices, std::vector<float> &k_sqr_distances,
                    unsigned int max_nn = 0) const
      {
        assert (index >= 0 && index < static_cast<int> (cloud.size ()) && "Out-of-bounds error in radiusSearch!");
        return (radiusSearch(cloud[index], radius, k_indices, k_sqr_distances, max_nn));
      }

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
      template <typename PointTDiff> inline int
      radiusSearchT (const PointTDiff &point, double radius, Indices &k_indices,
                     std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
      {
        PointT p;
        copyPoint (point, p);
        return (radiusSearch (p, radius, k_indices, k_sqr_distances, max_nn));
      }

      /** \brief Search for all the nearest neighbors of the query point in a given radius (zero-copy).
        *
        * \attention This method does not do any bounds checking for the input index
        * (i.e., index >= cloud.size () || index < 0), and assumes valid (i.e., finite) data.
        *
        * \param[in] index a \a valid index representing a \a valid query point in the dataset given
        * by \a setInputCloud. If indices were given in setInputCloud, index will be the position in
        * the indices vector.
        *
        * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
        * \param[out] k_indices the resultant indices of the neighboring points
        * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
        * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
        * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
        * returned.
        * \return number of neighbors found in radius
        *
        * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
        */
      virtual int
      radiusSearch (int index, double radius, Indices &k_indices,
                    std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
      {
        if (indices_ == nullptr)
        {
          assert (index >= 0 && index < static_cast<int> (input_->size ()) && "Out-of-bounds error in radiusSearch!");
          return (radiusSearch ((*input_)[index], radius, k_indices, k_sqr_distances, max_nn));
        }
        assert (index >= 0 && index < static_cast<int> (indices_->size ()) && "Out-of-bounds error in radiusSearch!");
        return (radiusSearch ((*input_)[(*indices_)[index]], radius, k_indices, k_sqr_distances, max_nn));
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

    protected:
      /** \brief The input point cloud dataset containing the points we need to use. */
      PointCloudConstPtr input_;

      /** \brief A pointer to the vector of point indices to use. */
      IndicesConstPtr indices_;

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
