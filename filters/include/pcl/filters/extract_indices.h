/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FILTERS_EXTRACT_INDICES_H_
#define PCL_FILTERS_EXTRACT_INDICES_H_

#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief @b ExtractIndices extracts a set of indices from a point cloud.
    * <br>
    * Usage examples:
    * \code
    * pcl::ExtractIndices<PointType> filter;
    * filter.setInputCloud (cloud_in);
    * filter.setIndices (indices_in);
    * // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
    * filter.filter (*cloud_out);
    * // Retrieve indices to all points in cloud_in except those referenced by indices_in:
    * filter.setNegative (true);
    * filter.filter (*indices_out);
    * // The resulting cloud_out is identical to cloud_in, but all points referenced by indices_in are made NaN:
    * filter.setNegative (true);
    * filter.setKeepOrganized (true);
    * filter.filter (*cloud_out);
    * \endcode
    * \note Does not inherently remove NaNs from results, hence the \a extract_removed_indices_ system is not used.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<typename PointT>
  class ExtractIndices : public FilterIndices<PointT>
  {
    public:
      typedef typename Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \brief Empty constructor. */
      ExtractIndices ()
      {
        use_indices_ = true;
        filter_name_ = "ExtractIndices";
      }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using PCLBase<PointT>::use_indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;

      /** \brief Extract point indices into a separate PointCloud
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Extract point indices
        * \param indices the resultant indices
        */
      void
      applyFilter (std::vector<int> &indices);
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ExtractIndices extracts a set of indices from a point cloud.
    * <br>
    * Usage examples:
    * \code
    * pcl::ExtractIndices<PointType> filter;
    * filter.setInputCloud (cloud_in);
    * filter.setIndices (indices_in);
    * // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
    * filter.filter (*cloud_out);
    * // Retrieve indices to all points in cloud_in except those referenced by indices_in:
    * filter.setNegative (true);
    * filter.filter (*indices_out);
    * // The resulting cloud_out is identical to cloud_in, but all points referenced by indices_in are made NaN:
    * filter.setNegative (true);
    * filter.setKeepOrganized (true);
    * filter.filter (*cloud_out);
    * \endcode
    * \note Does not inherently remove NaNs from results, hence the \a extract_removed_indices_ system is not used.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS ExtractIndices<sensor_msgs::PointCloud2> : public FilterIndices<sensor_msgs::PointCloud2>
  {
    public:
      typedef sensor_msgs::PointCloud2 PointCloud2;
      typedef PointCloud2::Ptr PointCloud2Ptr;
      typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

      /** \brief Empty constructor. */
      ExtractIndices ()
      {
        use_indices_ = true;
        filter_name_ = "ExtractIndices";
      }

    protected:
      using PCLBase<PointCloud2>::input_;
      using PCLBase<PointCloud2>::indices_;
      using PCLBase<PointCloud2>::use_indices_;
      using Filter<PointCloud2>::filter_name_;
      using Filter<PointCloud2>::getClassName;
      using FilterIndices<PointCloud2>::negative_;
      using FilterIndices<PointCloud2>::keep_organized_;
      using FilterIndices<PointCloud2>::user_filter_value_;

      /** \brief Extract point indices into a separate PointCloud
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PointCloud2 &output);

      /** \brief Extract point indices
        * \param indices the resultant indices
        */
      void
      applyFilter (std::vector<int> &indices);
  };
}

#endif  //#ifndef PCL_FILTERS_EXTRACT_INDICES_H_

