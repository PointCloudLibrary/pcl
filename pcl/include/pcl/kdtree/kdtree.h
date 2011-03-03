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
 * $Id: kdtree.h 35873 2011-02-09 00:58:01Z rusu $
 *
 */

#ifndef PCL_KDTREE_KDTREE_H_
#define PCL_KDTREE_KDTREE_H_

#include <limits.h>
#include "pcl/ros_macros.h"
#include "pcl/pcl_macros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include <boost/make_shared.hpp>

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b KdTree represents the base spatial locator class for nearest neighbor estimation. All types of spatial
    * locators should inherit from KdTree.
    * \author Radu Bogdan Rusu, Bastian Steder, Michael Dixon
    */
  template <typename PointT>
  class KdTree
  {
    typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
    typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;

    public:
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef boost::shared_ptr<PointCloud> PointCloudPtr;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      typedef pcl::PointRepresentation<PointT> PointRepresentation;
      //typedef boost::shared_ptr<PointRepresentation> PointRepresentationPtr;
      typedef boost::shared_ptr<const PointRepresentation> PointRepresentationConstPtr;

      // Boost shared pointers
      typedef boost::shared_ptr<KdTree<PointT> > Ptr;
      typedef boost::shared_ptr<const KdTree<PointT> > ConstPtr;

      /** \brief Empty constructor for KdTree. Sets some internal values to their defaults. */
      KdTree (bool sorted = true) : input_(), indices_(), 
                                    epsilon_(0.0), min_pts_(1), sorted_(sorted)
      {
        point_representation_ = boost::make_shared<DefaultPointRepresentation<PointT> > ();
      };


      /** \brief Provide a pointer to the input dataset.
        * \param cloud the const boost shared pointer to a PointCloud message
        * \param indices the point indices subset that is to be used from \a cloud - if NULL the whole point cloud is used
        */
      virtual inline void
      setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr ())
      {
        input_   = cloud;
        indices_ = indices;
      }

      /** \brief Get a pointer to the vector of indices used. */
      inline IndicesConstPtr const
      getIndices ()
      {
        return (indices_);
      }

      /** \brief Get a pointer to the input point cloud dataset. */
      inline PointCloudConstPtr
      getInputCloud ()
      {
        return (input_);
      }

      /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors. 
        * \param point_representation the const boost shared pointer to a PointRepresentation
        */
      inline void
      setPointRepresentation (const PointRepresentationConstPtr &point_representation)
      {
        point_representation_ = point_representation;
        setInputCloud (input_, indices_);  // Makes sense in derived classes to reinitialize the tree
      }

      /** \brief Get a pointer to the point representation used when converting points into k-D vectors. */
      inline PointRepresentationConstPtr const
      getPointRepresentation ()
      {
        return (point_representation_);
      }

      /** \brief Destructor for KdTree. Deletes all allocated data arrays and destroys the kd-tree structures. */
      virtual ~KdTree () {};

      /** \brief Search for k-nearest neighbors for the given query point.
        * \param cloud the point cloud data
        * \param index the index in \a cloud representing the query point
        * \param k the number of neighbors to search for
        * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        * \return number of neighbors found
        */
      virtual int 
      nearestKSearch (const PointCloud &cloud, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) = 0;

      /** \brief Search for k-nearest neighbors for the given query point.
        * \param point the given query point
        * \param k the number of neighbors to search for
        * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        * \return number of neighbors found
        */
      virtual int 
      nearestKSearch (const PointT &p_q, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) = 0;

      /** \brief Search for k-nearest neighbors for the given query point (zero-copy).
        * \param index the index representing the query point in the dataset given by \a setInputCloud
        *        if indices were given in setInputCloud, index will be the position in the indices vector
        * \param k the number of neighbors to search for
        * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
        * a priori!)
        * \return number of neighbors found
        */
      virtual int 
      nearestKSearch (int index, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) = 0;

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
        * \param cloud the point cloud data
        * \param index the index in \a cloud representing the query point
        * \param radius the radius of the sphere bounding all of p_q's neighbors
        * \param k_indices the resultant indices of the neighboring points
        * \param k_sqr_distances the resultant squared distances to the neighboring points
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
      virtual int 
      radiusSearch (const PointCloud &cloud, int index, double radius, std::vector<int> &k_indices,
                                 std::vector<float> &k_sqr_distances, int max_nn = INT_MAX) const = 0;

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
        * \param point the given query point
        * \param radius the radius of the sphere bounding all of p_q's neighbors
        * \param k_indices the resultant indices of the neighboring points
        * \param k_sqr_distances the resultant squared distances to the neighboring points
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
      virtual int 
      radiusSearch (const PointT &p_q, double radius, std::vector<int> &k_indices,
                                 std::vector<float> &k_sqr_distances, int max_nn = INT_MAX) const = 0;

      /** \brief Search for all the nearest neighbors of the query point in a given radius (zero-copy).
        * \param index the index representing the query point in the dataset given by \a setInputCloud
        *        if indices were given in setInputCloud, index will be the position in the indices vector
        * \param radius the radius of the sphere bounding all of p_q's neighbors
        * \param k_indices the resultant indices of the neighboring points
        * \param k_sqr_distances the resultant squared distances to the neighboring points
        * \param max_nn if given, bounds the maximum returned neighbors to this value
        * \return number of neighbors found in radius
        */
      virtual int 
      radiusSearch (int index, double radius, std::vector<int> &k_indices,
                                 std::vector<float> &k_sqr_distances, int max_nn = INT_MAX) const = 0;

      /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
        * \param eps precision (error bound) for nearest neighbors searches
        */
      inline void
      setEpsilon (double eps)
      {
        epsilon_ = eps;
      }

      /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
      inline double
      getEpsilon ()
      {
        return (epsilon_);
      }

      /** \brief Minimum allowed number of k nearest neighbors points that a viable result must contain. */
      inline void
      setMinPts (int min_pts)
      {
        min_pts_ = min_pts;
      }

      /** \brief Get the minimum allowed number of k nearest neighbors points that a viable result must contain. */
      inline float
      getMinPts ()
      {
        return (min_pts_);
      }

    protected:
      /** \brief The input point cloud dataset containing the points we need to use. */
      PointCloudConstPtr input_;

      /** \brief A pointer to the vector of point indices to use. */
      IndicesConstPtr indices_;

      /** \brief Epsilon precision (error bound) for nearest neighbors searches. */
      double epsilon_;

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

#endif  //#ifndef _PCL_KDTREE_KDTREE_H_
