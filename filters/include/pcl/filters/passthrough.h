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
 * $Id$
 *
 */

#ifndef PCL_FILTERS_PASSTHROUGH_H_
#define PCL_FILTERS_PASSTHROUGH_H_

#include <pcl/filters/filter.h>

namespace pcl
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief PassThrough uses the base Filter class methods to pass through all data that satisfies the user given
    * constraints.
    * \author Radu B. Rusu
    * \ingroup filters
    */
  template<typename PointT>
  class PassThrough : public Filter<PointT>
  {
    using Filter<PointT>::input_;
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;

    using Filter<PointT>::removed_indices_;
    using Filter<PointT>::extract_removed_indices_;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Constructor. */
      PassThrough (bool extract_removed_indices = false) :
        Filter<PointT>::Filter (extract_removed_indices), keep_organized_ (false), 
        user_filter_value_ (std::numeric_limits<float>::quiet_NaN ()),
        filter_field_name_ (""), 
        filter_limit_min_ (-FLT_MAX), filter_limit_max_ (FLT_MAX),
        filter_limit_negative_ (false)
      {
        filter_name_ = "PassThrough";
      }

      /** \brief Set whether the filtered points should be kept and set to the
        * value given through \a setUserFilterValue (default: NaN), or removed
        * from the PointCloud, thus potentially breaking its organized
        * structure. By default, points are removed.
        *
        * \param[in] val set to true whether the filtered points should be kept and
        * set to a given user value (default: NaN)
        */
      inline void
      setKeepOrganized (bool val)
      {
        keep_organized_ = val;
      }

      /** \brief Obtain the value of the internal \a keep_organized_ parameter. */
      inline bool
      getKeepOrganized ()
      {
        return (keep_organized_);
      }

      /** \brief Provide a value that the filtered points should be set to
        * instead of removing them.  Used in conjunction with \a
        * setKeepOrganized ().
        * \param[in] val the user given value that the filtered point dimensions should be set to
        */
      inline void
      setUserFilterValue (float val)
      {
        user_filter_value_ = val;
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
      getFilterFieldName ()
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
      getFilterLimits (double &limit_min, double &limit_max)
      {
        limit_min = filter_limit_min_;
        limit_max = filter_limit_max_;
      }

      /** \brief Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max).
        * Default: false.
        * \param[in] limit_negative return data inside the interval (false) or outside (true)
        */
      inline void
      setFilterLimitsNegative (const bool limit_negative)
      {
        filter_limit_negative_ = limit_negative;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
        * \param[out] limit_negative true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline void
      getFilterLimitsNegative (bool &limit_negative)
      {
        limit_negative = filter_limit_negative_;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
        * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
       inline bool
      getFilterLimitsNegative ()
      {
        return (filter_limit_negative_);
      }

    protected:
      /** \brief Filter a Point Cloud.
        * \param[out] output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);

      typedef typename pcl::traits::fieldList<PointT>::type FieldList;

    private:
      /** \brief Keep the structure of the data organized, by setting the
        * filtered points to the a user given value (NaN by default). 
        */
      bool keep_organized_;

      /** \brief User given value to be set to any filtered point. Casted to
        * the correct field type. 
        */
      float user_filter_value_;

      /** \brief The desired user filter field name. */
      std::string filter_field_name_;

      /** \brief The minimum allowed filter value a point will be considered from. */
      double filter_limit_min_;

      /** \brief The maximum allowed filter value a point will be considered from. */
      double filter_limit_max_;

      /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
      bool filter_limit_negative_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief PassThrough uses the base Filter class methods to pass through all data that satisfies the user given
    * constraints.
    * \author Radu B. Rusu
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS PassThrough<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    using Filter<sensor_msgs::PointCloud2>::removed_indices_;
    using Filter<sensor_msgs::PointCloud2>::extract_removed_indices_;

    public:
      /** \brief Constructor. */
      PassThrough (bool extract_removed_indices = false) :
        Filter<sensor_msgs::PointCloud2>::Filter (extract_removed_indices), keep_organized_ (false),
        user_filter_value_ (std::numeric_limits<float>::quiet_NaN ()),
        filter_field_name_ (""), filter_limit_min_ (-FLT_MAX), filter_limit_max_ (FLT_MAX),
        filter_limit_negative_ (false)
      {
        filter_name_ = "PassThrough";
      }

      /** \brief Set whether the filtered points should be kept and set to the
        * value given through \a setUserFilterValue (default: NaN), or removed
        * from the PointCloud, thus potentially breaking its organized
        * structure. By default, points are removed.
        *
        * \param[in] val set to true whether the filtered points should be kept and
        * set to a given user value (default: NaN)
        */
      inline void
      setKeepOrganized (bool val)
      {
        keep_organized_ = val;
      }

      /** \brief Obtain the value of the internal \a keep_organized_ parameter. */
      inline bool
      getKeepOrganized ()
      {
        return (keep_organized_);
      }

      /** \brief Provide a value that the filtered points should be set to
        * instead of removing them.  Used in conjunction with \a
        * setKeepOrganized ().
        * \param[in] val the user given value that the filtered point dimensions should be set to
        */
      inline void
      setUserFilterValue (float val)
      {
        user_filter_value_ = val;
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
      getFilterFieldName ()
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
      getFilterLimits (double &limit_min, double &limit_max)
      {
        limit_min = filter_limit_min_;
        limit_max = filter_limit_max_;
      }

      /** \brief Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max).
        * Default: false.
        * \param[in] limit_negative return data inside the interval (false) or outside (true)
        */
      inline void
      setFilterLimitsNegative (const bool limit_negative)
      {
        filter_limit_negative_ = limit_negative;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
        * \param[out] limit_negative true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline void
      getFilterLimitsNegative (bool &limit_negative)
      {
        limit_negative = filter_limit_negative_;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
        * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline bool
      getFilterLimitsNegative ()
      {
        return (filter_limit_negative_);
      }

    protected:
      void
      applyFilter (PointCloud2 &output);

    private:
      /** \brief Keep the structure of the data organized, by setting the
        * filtered points to the a user given value (NaN by default). 
        */
      bool keep_organized_;

      /** \brief User given value to be set to any filtered point. Casted to
        * the correct field type. 
        */
      float user_filter_value_;

      /** \brief The desired user filter field name. */
      std::string filter_field_name_;

      /** \brief The minimum allowed filter value a point will be considered from. */
      double filter_limit_min_;

      /** \brief The maximum allowed filter value a point will be considered from. */
      double filter_limit_max_;

      /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
      bool filter_limit_negative_;

  };
}

#endif  //#ifndef PCL_FILTERS_PASSTHROUGH_H_
