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

#include <cfloat> // for FLT_MIN, FLT_MAX
#include <pcl/pcl_macros.h>
#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief @b PassThrough passes points in a cloud based on constraints for one particular field of the point type.
    * \details Iterates through the entire input once, automatically filtering non-finite points and the points outside
    * the interval specified by setFilterLimits(), which applies only to the field specified by setFilterFieldName().
    * <br><br>
    * Usage example:
    * \code
    * pcl::PassThrough<PointType> ptfilter (true); // Initializing with true will allow us to extract the removed indices
    * ptfilter.setInputCloud (cloud_in);
    * ptfilter.setFilterFieldName ("x");
    * ptfilter.setFilterLimits (0.0, 1000.0);
    * ptfilter.filter (*indices_x);
    * // The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
    * indices_rem = ptfilter.getRemovedIndices ();
    * // The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
    * // and also indexes all non-finite points of cloud_in
    * ptfilter.setIndices (indices_x);
    * ptfilter.setFilterFieldName ("z");
    * ptfilter.setFilterLimits (-10.0, 10.0);
    * ptfilter.setNegative (true);
    * ptfilter.filter (*indices_xz);
    * // The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0
    * ptfilter.setIndices (indices_xz);
    * ptfilter.setFilterFieldName ("intensity");
    * ptfilter.setFilterLimits (FLT_MIN, 0.5);
    * ptfilter.setNegative (false);
    * ptfilter.filter (*cloud_out);
    * // The resulting cloud_out contains all points of cloud_in that are finite and have:
    * // x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and intensity smaller than 0.5.
    * \endcode
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */
  template <typename PointT>
  class PassThrough : public FilterIndices<PointT>
  {
    protected:
      using PointCloud = typename FilterIndices<PointT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;
      using FieldList = typename pcl::traits::fieldList<PointT>::type;

    public:

      using Ptr = shared_ptr<PassThrough<PointT> >;
      using ConstPtr = shared_ptr<const PassThrough<PointT> >;


      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
      PassThrough (bool extract_removed_indices = false) :
        FilterIndices<PointT> (extract_removed_indices),
        filter_field_name_ (""),
        filter_limit_min_ (FLT_MIN),
        filter_limit_max_ (FLT_MAX)
      {
        filter_name_ = "PassThrough";
      }

      /** \brief Provide the name of the field to be used for filtering data.
        * \details In conjunction with setFilterLimits(), points having values outside this interval for this field will be discarded.
        * \param[in] field_name The name of the field that will be used for filtering.
        */
      inline void
      setFilterFieldName (const std::string &field_name)
      {
        filter_field_name_ = field_name;
      }

      /** \brief Retrieve the name of the field to be used for filtering data.
        * \return The name of the field that will be used for filtering.
        */
      inline std::string const
      getFilterFieldName () const
      {
        return (filter_field_name_);
      }

      /** \brief Set the numerical limits for the field for filtering data.
        * \details In conjunction with setFilterFieldName(), points having values outside this interval for this field will be discarded.
        * \param[in] limit_min The minimum allowed field value (default = FLT_MIN).
        * \param[in] limit_max The maximum allowed field value (default = FLT_MAX).
        */
      inline void
      setFilterLimits (const float &limit_min, const float &limit_max)
      {
        filter_limit_min_ = limit_min;
        filter_limit_max_ = limit_max;
      }

      /** \brief Get the numerical limits for the field for filtering data.
        * \param[out] limit_min The minimum allowed field value (default = FLT_MIN).
        * \param[out] limit_max The maximum allowed field value (default = FLT_MAX).
        */
      inline void
      getFilterLimits (float &limit_min, float &limit_max) const
      {
        limit_min = filter_limit_min_;
        limit_max = filter_limit_max_;
      }

      /** \brief Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max)
        * Default: false.
        * \warning This method will be removed in the future. Use setNegative() instead.
        * \param[in] limit_negative return data inside the interval (false) or outside (true)
        */
      PCL_DEPRECATED(1, 13, "use inherited FilterIndices::setNegative() instead")
      inline void
      setFilterLimitsNegative (const bool limit_negative)
      {
        negative_ = limit_negative;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false).
        * \warning This method will be removed in the future. Use getNegative() instead.
        * \param[out] limit_negative true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      PCL_DEPRECATED(1, 13, "use inherited FilterIndices::getNegative() instead")
      inline void
      getFilterLimitsNegative (bool &limit_negative) const
      {
        limit_negative = negative_;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false).
        * \warning This method will be removed in the future. Use getNegative() instead.
        * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline bool
      getFilterLimitsNegative () const
      {
        return (negative_);
      }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

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

    private:
      /** \brief The name of the field that will be used for filtering. */
      std::string filter_field_name_;

      /** \brief The minimum allowed field value (default = FLT_MIN). */
      float filter_limit_min_;

      /** \brief The maximum allowed field value (default = FLT_MIN). */
      float filter_limit_max_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief PassThrough uses the base Filter class methods to pass through all data that satisfies the user given
    * constraints.
    * \author Radu B. Rusu
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS PassThrough<pcl::PCLPointCloud2> : public FilterIndices<pcl::PCLPointCloud2>
  {
    using PCLPointCloud2 = pcl::PCLPointCloud2;
    using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
    using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

    using Filter<pcl::PCLPointCloud2>::removed_indices_;
    using Filter<pcl::PCLPointCloud2>::extract_removed_indices_;

    public:
      /** \brief Constructor. */
      PassThrough (bool extract_removed_indices = false) :
        FilterIndices<pcl::PCLPointCloud2>::FilterIndices (extract_removed_indices),
        filter_field_name_ (""), filter_limit_min_ (-FLT_MAX), filter_limit_max_ (FLT_MAX)
      {
        filter_name_ = "PassThrough";
      }

      /** \brief Provide the name of the field to be used for filtering data. In conjunction with  \a setFilterLimits,
        * points having values outside this interval will be discarded.
        * \param[in] field_name the name of the field that contains values used for filtering
        */
      inline void
      setFilterFieldName (const std::string &field_name)
      {
        filter_field_name_ = field_name;
      }

      /** \brief Get the name of the field used for filtering. */
      inline std::string const
      getFilterFieldName () const
      {
        return (filter_field_name_);
      }

      /** \brief Set the field filter limits. All points having field values outside this interval will be discarded.
        * \param[in] limit_min the minimum allowed field value
        * \param[in] limit_max the maximum allowed field value
        */
      inline void
      setFilterLimits (const double &limit_min, const double &limit_max)
      {
        filter_limit_min_ = limit_min;
        filter_limit_max_ = limit_max;
      }

      /** \brief Get the field filter limits (min/max) set by the user. The default values are -FLT_MAX, FLT_MAX.
        * \param[out] limit_min the minimum allowed field value
        * \param[out] limit_max the maximum allowed field value
        */
      inline void
      getFilterLimits (double &limit_min, double &limit_max) const
      {
        limit_min = filter_limit_min_;
        limit_max = filter_limit_max_;
      }

    protected:
      void
      applyFilter (PCLPointCloud2 &output) override;

      void
      applyFilter (Indices &indices) override;

    private:
      /** \brief The desired user filter field name. */
      std::string filter_field_name_;

      /** \brief The minimum allowed filter value a point will be considered from. */
      double filter_limit_min_;

      /** \brief The maximum allowed filter value a point will be considered from. */
      double filter_limit_max_;

  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/passthrough.hpp>
#endif
