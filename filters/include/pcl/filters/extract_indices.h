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

#pragma once

#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief @b ExtractIndices extracts a set of indices from a point cloud.
    * \details Usage example:
    * \code
    * pcl::ExtractIndices<PointType> eifilter (true); // Initializing with true will allow us to extract the removed indices
    * eifilter.setInputCloud (cloud_in);
    * eifilter.setIndices (indices_in);
    * eifilter.filter (*cloud_out);
    * // The resulting cloud_out contains all points of cloud_in that are indexed by indices_in
    * indices_rem = eifilter.getRemovedIndices ();
    * // The indices_rem array indexes all points of cloud_in that are not indexed by indices_in
    * eifilter.setNegative (true);
    * eifilter.filter (*indices_out);
    * // Alternatively: the indices_out array is identical to indices_rem
    * eifilter.setNegative (false);
    * eifilter.setUserFilterValue (1337.0);
    * eifilter.filterDirectly (cloud_in);
    * // This will directly modify cloud_in instead of creating a copy of the cloud
    * // It will overwrite all fields of the filtered points by the user value: 1337
    * \endcode
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template<typename PointT>
  class ExtractIndices : public FilterIndices<PointT>
  {
    protected:
      using PointCloud = typename FilterIndices<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;
      using FieldList = typename pcl::traits::fieldList<PointT>::type;

    public:

      using Ptr = shared_ptr<ExtractIndices<PointT> >;
      using ConstPtr = shared_ptr<const ExtractIndices<PointT> >;

      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
      ExtractIndices (bool extract_removed_indices = false) :
        FilterIndices<PointT>::FilterIndices (extract_removed_indices)
      {
        use_indices_ = true;
        filter_name_ = "ExtractIndices";
      }

      /** \brief Apply the filter and store the results directly in the input cloud.
        * \details This method will save the time and memory copy of an output cloud but can not alter the original size of the input cloud:
        * It operates as though setKeepOrganized() is true and will overwrite the filtered points instead of remove them.
        * All fields of filtered points are replaced with the value set by setUserFilterValue() (default = NaN).
        * This method also automatically alters the input cloud set via setInputCloud().
        * It does not alter the value of the internal keep organized boolean as set by setKeepOrganized().
        * \param cloud The point cloud used for input and output.
        */
      void
      filterDirectly (PointCloudPtr &cloud);

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using PCLBase<PointT>::use_indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Filtered results are stored in a separate point cloud.
        * \param[out] output The resultant point cloud.
        */
      void
      applyFilter (PointCloud &output) override;

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilter (Indices &indices) override
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilterIndices (Indices &indices);
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
  class PCL_EXPORTS ExtractIndices<pcl::PCLPointCloud2> : public FilterIndices<pcl::PCLPointCloud2>
  {
    public:
      using PCLPointCloud2 = pcl::PCLPointCloud2;
      using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
      using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

      /** \brief Empty constructor. */
      ExtractIndices ()
      {
        use_indices_ = true;
        filter_name_ = "ExtractIndices";
      }

    protected:
      using PCLBase<PCLPointCloud2>::input_;
      using PCLBase<PCLPointCloud2>::indices_;
      using PCLBase<PCLPointCloud2>::use_indices_;
      using Filter<PCLPointCloud2>::filter_name_;
      using Filter<PCLPointCloud2>::getClassName;
      using FilterIndices<PCLPointCloud2>::negative_;
      using FilterIndices<PCLPointCloud2>::keep_organized_;
      using FilterIndices<PCLPointCloud2>::user_filter_value_;

      /** \brief Extract point indices into a separate PointCloud
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PCLPointCloud2 &output) override;

      /** \brief Extract point indices
        * \param indices the resultant indices
        */
      void
      applyFilter (Indices &indices) override;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/extract_indices.hpp>
#endif
