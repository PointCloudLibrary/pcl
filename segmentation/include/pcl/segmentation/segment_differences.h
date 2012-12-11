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

#ifndef PCL_SEGMENT_DIFFERENCES_H_
#define PCL_SEGMENT_DIFFERENCES_H_

#include <pcl/pcl_base.h>
#include <pcl/search/pcl_search.h>

namespace pcl
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Obtain the difference between two aligned point clouds as another point cloud, given a distance threshold.
    * \param src the input point cloud source
    * \param tgt the input point cloud target we need to obtain the difference against
    * \param threshold the distance threshold (tolerance) for point correspondences. (e.g., check if f a point p1 from 
    * src has a correspondence > threshold than a point p2 from tgt)
    * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching built over \a tgt
    * \param output the resultant output point cloud difference
    * \ingroup segmentation
    */
  template <typename PointT> 
  void getPointCloudDifference (
      const pcl::PointCloud<PointT> &src, const pcl::PointCloud<PointT> &tgt, 
      double threshold, const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
      pcl::PointCloud<PointT> &output);

  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b SegmentDifferences obtains the difference between two spatially
    * aligned point clouds and returns the difference between them for a maximum
    * given distance threshold.
    * \author Radu Bogdan Rusu
    * \ingroup segmentation
    */
  template <typename PointT>
  class SegmentDifferences: public PCLBase<PointT>
  {
    typedef PCLBase<PointT> BasePCLBase;

    public:
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef typename pcl::search::Search<PointT> KdTree;
      typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      /** \brief Empty constructor. */
      SegmentDifferences () : 
        tree_ (), target_ (), distance_threshold_ (0)
      {};

      /** \brief Provide a pointer to the target dataset against which we
        * compare the input cloud given in setInputCloud
        *
        * \param cloud the target PointCloud dataset
        */
      inline void 
      setTargetCloud (const PointCloudConstPtr &cloud) { target_ = cloud; }

      /** \brief Get a pointer to the input target point cloud dataset. */
      inline PointCloudConstPtr const 
      getTargetCloud () { return (target_); }

      /** \brief Provide a pointer to the search object.
        * \param tree a pointer to the spatial search object.
        */
      inline void 
      setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr 
      getSearchMethod () { return (tree_); }

      /** \brief Set the maximum distance tolerance (squared) between corresponding
        * points in the two input datasets.
        *
        * \param sqr_threshold the squared distance tolerance as a measure in L2 Euclidean space
        */
      inline void 
      setDistanceThreshold (double sqr_threshold) { distance_threshold_ = sqr_threshold; }

      /** \brief Get the squared distance tolerance between corresponding points as a
        * measure in the L2 Euclidean space.
        */
      inline double 
      getDistanceThreshold () { return (distance_threshold_); }

      /** \brief Segment differences between two input point clouds.
        * \param output the resultant difference between the two point clouds as a PointCloud
        */
      void 
      segment (PointCloud &output);

    protected:
      // Members derived from the base class
      using BasePCLBase::input_;
      using BasePCLBase::indices_;
      using BasePCLBase::initCompute;
      using BasePCLBase::deinitCompute;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The input target point cloud dataset. */
      PointCloudConstPtr target_;

      /** \brief The distance tolerance (squared) as a measure in the L2
        * Euclidean space between corresponding points. 
        */
      double distance_threshold_;

      /** \brief Class getName method. */
      virtual std::string 
      getClassName () const { return ("SegmentDifferences"); }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/segment_differences.hpp>
#endif

#endif  //#ifndef PCL_SEGMENT_DIFFERENCES_H_
