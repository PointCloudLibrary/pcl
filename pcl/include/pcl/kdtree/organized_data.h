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
 * $Id: organized_data.h 35648 2011-02-01 01:08:27Z michael.s.dixon $
 *
 */

#ifndef PCL_KDTREE_ORGANIZED_DATA_H_
#define PCL_KDTREE_ORGANIZED_DATA_H_

#include "pcl/kdtree/kdtree.h"
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

namespace pcl
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b OrganizedDataIndex is a type of spatial locator used to query organized datasets, such as point clouds
    * acquired using dense stereo devices. The class best supports square data blocks for now in the form of (k*k+1)^2.
    * \author Radu Bogdan Rusu
    */
  template <typename PointT>
  class OrganizedDataIndex : public KdTree<PointT>
  {
    using KdTree<PointT>::input_;
    using KdTree<PointT>::min_pts_;

    typedef KdTree<PointT> BaseClass;
    typedef typename KdTree<PointT>::PointCloud PointCloud;
    typedef typename KdTree<PointT>::PointCloudConstPtr PointCloudConstPtr;

    public:
      // Boost shared pointers
      typedef boost::shared_ptr<OrganizedDataIndex<PointT> > Ptr;
      typedef boost::shared_ptr<const OrganizedDataIndex<PointT> > ConstPtr;

      /** \brief Empty constructor for OrganizedDataIndex. Sets some internal values to their defaults. */
      OrganizedDataIndex () : KdTree<PointT>(), max_distance_ (0), horizontal_window_ (0), vertical_window_ (0) {}

      inline Ptr makeShared () const { return Ptr (new OrganizedDataIndex<PointT> (*this)); } 

      // The following methods will not be implemented
      int nearestKSearch (const PointT &p_q, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
      {
        ROS_ERROR ("%s: Method not implemented!", __PRETTY_FUNCTION__); return (false);
      }
      int radiusSearch (const PointT &p_q, double radius, std::vector<int> &k_indices,
                                std::vector<float> &k_distances, int max_nn = INT_MAX) const
      {
        ROS_ERROR ("%s: Method not implemented!", __PRETTY_FUNCTION__); return (false);
      }

      /** \brief Approximate search for neighbors around the given query point within radius.
        * \param cloud the point cloud data.
        * \param index the index in \a cloud representing the query point.
        * \param radius the maximum distance to search for neighbors in.
        * \param k_indices the resultant point indices
        * \param k_distances the resultant !squared! point distances
        * \param max_nn maximum number of points to return
        */
      int radiusSearch (const PointCloud &cloud, int index, double radius, std::vector<int> &k_indices,
                         std::vector<float> &k_distances, int max_nn = INT_MAX) const;

      /** \brief Approximate search for neighbors around the point from the given index within radius.
        * \param index the index in \a cloud representing the query point.
        * \param radius the maximum distance to search for neighbors in.
        * \param k_indices the resultant point indices
        * \param k_distances the resultant !squared! point distances
        * \param max_nn maximum number of points to return
        */
      inline int 
      radiusSearch (int index, double radius, std::vector<int> &k_indices, 
                    std::vector<float> &k_distances, int max_nn = INT_MAX) const
      {
        if (!input_)
        {
          ROS_ERROR ("[%s] Input dataset does not exist or wrong input dataset!", __PRETTY_FUNCTION__);
          return (false);
        }
        return (radiusSearch (*input_, index, radius, k_indices, k_distances, max_nn));
      }

      /** \brief Search for k-nearest neighbors for the given query point.
        * \param cloud the point cloud data
        * \param index the index in \a cloud representing the query point
        * \param k the number of neighbors to search for (not used)
        * \param k_indices the resultant point indices (must be resized to \a k beforehand!)
        * \param k_distances \note this function does not return distances
        */
      int nearestKSearch (const PointCloud &cloud, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);

      /** \brief Search for k-nearest neighbors for the given query point.
        * \param index the index representing the query point
        * \param k the number of neighbors to search for (not used)
        * \param k_indices the resultant point indices (must be resized to \a k beforehand!)
        * \param k_distances \note this function does not return distances
        */
      inline int 
      nearestKSearch (int index, int k, std::vector<int> &k_indices, 
                      std::vector<float> &k_distances)
      {
        if (!input_)
        {
          ROS_ERROR ("[pcl::%s::nearestKSearch] Input dataset does not exist or wrong input dataset!", getName ().c_str ());
          return 0;
        }
        return (nearestKSearch (*input_, index, k, k_indices, k_distances));
      }

      /** \brief Set the search window (horizontal, vertical) in pixels.
        * \param horizontal the horizontal window
        * \param vertical the vertical window
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
      void setSearchWindowAsK (int k);

      /** \brief Get the horizontal search window in pixels. */
      int getHorizontalSearchWindow () const { return (horizontal_window_); }

      /** \brief Get the vertical search window in pixels. */
      int getVerticalSearchWindow () const { return (vertical_window_); }

      /** \brief Set the maximum allowed distance between the query point and its k-nearest neighbors. */
      void setMaxDistance (float max_dist) { max_distance_ = max_dist; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the maximum allowed distance between the query point and its k-nearest neighbors. */
      float getMaxDistance () const { return (max_distance_); }

    private:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Class getName method. */
      virtual std::string getName () const { return ("OrganizedDataIndex"); }

      /** \brief Maximum allowed distance between the query point and its k-neighbors. */
      float max_distance_;

      /** \brief The horizontal search window. */
      int horizontal_window_;

      /** \brief The horizontal search window. */
      int vertical_window_;
  };
}

#endif  //#ifndef _PCL_KDTREE_ORGANIZED_DATA_H_
