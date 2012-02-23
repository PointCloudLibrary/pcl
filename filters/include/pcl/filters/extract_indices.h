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
 * $Id$
 *
 */

#ifndef PCL_FILTERS_EXTRACTINDICES_H_
#define PCL_FILTERS_EXTRACTINDICES_H_

#include "pcl/filters/filter.h"

namespace pcl
{
  /** \brief @b ExtractIndices extracts a set of indices from a PointCloud as a separate PointCloud.
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<typename PointT>
  class ExtractIndices : public Filter<PointT>
  {
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;
    using Filter<PointT>::use_indices_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::input_;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      ExtractIndices () :
        negative_ (false)
      {
        use_indices_ = true;
        filter_name_ = "ExtractIndices";
      }

      /** \brief Set whether the indices should be returned, or all points _except_ the indices.
        * \param negative true if all points _except_ the input indices will be returned, false otherwise
        */
      inline void
      setNegative (bool negative)
      {
        negative_ = negative;
      }

      /** \brief Get the value of the internal \a negative parameter. If true, all points _except_ the input indices
        * will be returned. 
        */
      inline bool
      getNegative ()
      {
        return (negative_);
      }

    protected:
      /** \brief Extract point indices into a separate PointCloud
        * \param output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);

      /** \brief If true, all the points _except_ the input indices will be returned. False by default. */
      bool negative_;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ExtractIndices extracts a set of indices from a PointCloud as a separate PointCloud.
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS ExtractIndices<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    using Filter<sensor_msgs::PointCloud2>::filter_name_;
    using Filter<sensor_msgs::PointCloud2>::getClassName;

    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
      /** \brief Empty constructor. */
      ExtractIndices () : negative_ (false)
      {
        use_indices_ = true;
        filter_name_ = "ProjectInliers";
      }

      /** \brief Set whether the indices should be returned, or all points _except_ the indices.
        * \param negative true if all points _except_ the input indices will be returned, false otherwise
        */
      inline void
      setNegative (bool negative)
      {
        negative_ = negative;
      }

      /** \brief Get the value of the internal \a negative parameter. If true, all points _except_ the input indices
        * will be returned. 
        */
      inline bool
      getNegative ()
      {
        return (negative_);
      }

    protected:
      void
      applyFilter (PointCloud2 &output);

      /** \brief If true, all the points _except_ the input indices will be returned. False by default. */
      bool negative_;
  };
}

#endif  //#ifndef PCL_FILTERS_EXTRACTINDICES_H_
