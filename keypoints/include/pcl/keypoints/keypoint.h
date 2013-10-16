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

#ifndef PCL_KEYPOINT_H_
#define PCL_KEYPOINT_H_

// PCL includes
#include <pcl/pcl_base.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <pcl/search/pcl_search.h>
#include <pcl/pcl_config.h>

namespace pcl
{
  /** \brief @b Keypoint represents the base class for key points.
    * \author Bastian Steder
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT>
  class Keypoint : public PCLBase<PointInT>
  {
    public:
      typedef boost::shared_ptr<Keypoint<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const Keypoint<PointInT, PointOutT> > ConstPtr;

      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::input_;

      typedef PCLBase<PointInT> BaseClass;
      typedef typename pcl::search::Search<PointInT> KdTree;
      typedef typename pcl::search::Search<PointInT>::Ptr KdTreePtr;
      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
      typedef pcl::PointCloud<PointOutT> PointCloudOut;
      typedef boost::function<int (int, double, std::vector<int> &, std::vector<float> &)> SearchMethod;
      typedef boost::function<int (const PointCloudIn &cloud, int index, double, std::vector<int> &, std::vector<float> &)> SearchMethodSurface;

    public:
      /** \brief Empty constructor. */
      Keypoint () : 
        BaseClass (), 
        name_ (),
        search_method_ (),
        search_method_surface_ (),
        surface_ (), 
        tree_ (), 
        search_parameter_ (0), 
        search_radius_ (0), 
        k_ (0) 
      {};
      
      /** \brief Empty destructor */
      virtual ~Keypoint () {}

      /** \brief Provide a pointer to the input dataset that we need to estimate features at every point for.
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      virtual void
      setSearchSurface (const PointCloudInConstPtr &cloud) { surface_ = cloud; }

      /** \brief Get a pointer to the surface point cloud dataset. */
      inline PointCloudInConstPtr
      getSearchSurface () { return (surface_); }

      /** \brief Provide a pointer to the search object.
        * \param tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr
      getSearchMethod () { return (tree_); }

      /** \brief Get the internal search parameter. */
      inline double
      getSearchParameter () { return (search_parameter_); }

      /** \brief Set the number of k nearest neighbors to use for the feature estimation.
        * \param k the number of k-nearest neighbors
        */
      inline void
      setKSearch (int k) { k_ = k; }

      /** \brief get the number of k nearest neighbors used for the feature estimation. */
      inline int
      getKSearch () { return (k_); }

      /** \brief Set the sphere radius that is to be used for determining the nearest neighbors used for the
       *         key point detection
        * \param radius the sphere radius used as the maximum distance to consider a point a neighbor
        */
      inline void
      setRadiusSearch (double radius) { search_radius_ = radius; }

      /** \brief Get the sphere radius used for determining the neighbors. */
      inline double
      getRadiusSearch () { return (search_radius_); }

      /** \brief \return the keypoints indices in the input cloud.
        * \note not all the daughter classes populate the keypoints indices so check emptiness before use.
        */
      pcl::PointIndicesConstPtr
      getKeypointsIndices () { return (keypoints_indices_); }

      /** \brief Base method for key point detection for all points given in <setInputCloud (), setIndices ()> using
        * the surface in setSearchSurface () and the spatial locator in setSearchMethod ()
        * \param output the resultant point cloud model dataset containing the estimated features
        */
      inline void
      compute (PointCloudOut &output);

      /** \brief Search for k-nearest neighbors using the spatial locator from \a setSearchmethod, and the given surface
        * from \a setSearchSurface.
        * \param index the index of the query point
        * \param parameter the search parameter (either k or radius)
        * \param indices the resultant vector of indices representing the k-nearest neighbors
        * \param distances the resultant vector of distances representing the distances from the query point to the
        * k-nearest neighbors
        */
      inline int
      searchForNeighbors (int index, double parameter, std::vector<int> &indices, std::vector<float> &distances) const
      {
        if (surface_ == input_)       // if the two surfaces are the same
          return (search_method_ (index, parameter, indices, distances));
        else
          return (search_method_surface_ (*input_, index, parameter, indices, distances));
      }

    protected:
      using PCLBase<PointInT>::deinitCompute;

      virtual bool
      initCompute ();

      /** \brief The key point detection method's name. */
      std::string name_;

      /** \brief The search method template for indices. */
      SearchMethod search_method_;

      /** \brief The search method template for points. */
      SearchMethodSurface search_method_surface_;

      /** \brief An input point cloud describing the surface that is to be used for nearest neighbors estimation. */
      PointCloudInConstPtr surface_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The actual search parameter (casted from either \a search_radius_ or \a k_). */
      double search_parameter_;

      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief The number of K nearest neighbors to use for each point. */
      int k_;

      /** \brief Indices of the keypoints in the input cloud. */
      pcl::PointIndicesPtr keypoints_indices_;

      /** \brief Get a string representation of the name of this class. */
      inline const std::string&
      getClassName () const { return (name_); }

      /** \brief Abstract key point detection method. */
      virtual void
      detectKeypoints (PointCloudOut &output) = 0;
  };
}

#include <pcl/keypoints/impl/keypoint.hpp>

#endif  //#ifndef PCL_KEYPOINT_H_
