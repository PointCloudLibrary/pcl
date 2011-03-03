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
 * $Id: passthrough.h 35664 2011-02-01 08:08:00Z rusu $
 *
 */

#ifndef PCL_FILTERS_PASSTHROUGH_H_
#define PCL_FILTERS_PASSTHROUGH_H_

#include "pcl/filters/filter.h"

namespace pcl
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b PassThrough uses the base Filter class methods to pass through all data that satisfies the user given
    * constraints.
    * \author Radu Bogdan Rusu
    */
  template <typename PointT>
  class PassThrough: public Filter<PointT>
  {
    using Filter<PointT>::input_;
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::filter_field_name_;
    using Filter<PointT>::filter_limit_min_;
    using Filter<PointT>::filter_limit_max_;
    using Filter<PointT>::filter_limit_negative_;
    using Filter<PointT>::getClassName;
    
    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      PassThrough () :
        keep_organized_ (false), user_filter_value_ (std::numeric_limits<float>::quiet_NaN ())
      {
        filter_name_ = "PassThrough";
      };

      /** \brief Set whether the filtered points should be kept and set to the
        * value given through \a setUserFilterValue (default: NaN), or removed
        * from the PointCloud, thus potentially breaking its organized
        * structure. By default, points are removed.
        *
        * \param val set to true whether the filtered points should be kept and
        * set to a given user value (default: NaN)
        */
      inline void setKeepOrganized (bool val) { keep_organized_ = val; }
      
      inline bool getKeepOrganized () { return (keep_organized_); }

      /** \brief Provide a value that the filtered points should be set to
        * instead of removing them.  Used in conjunction with \a
        * setKeepOrganized ().
        * \param val the user given value that the filtered point dimensions should be set to
        */
      inline void setUserFilterValue (float val) { user_filter_value_ = val; }
    protected:
      /** \brief Filter a Point Cloud.
        * \param output the resultant point cloud message
        */
      void applyFilter (PointCloud &output);

      typedef typename pcl::traits::fieldList<PointT>::type FieldList;

    private:
      /** \brief Keep the structure of the data organized, by setting the
       * filtered points to the a user given value (NaN by default). */
      bool keep_organized_;

      /** \brief User given value to be set to any filtered point. Casted to
       * the correct field type. */
      float user_filter_value_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b PassThrough uses the base Filter class methods to pass through all data that satisfies the user given
    * constraints.
    * \author Radu Bogdan Rusu
    */
  template <>
  class PassThrough<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
      /** \brief Empty constructor. */
      PassThrough () :
        keep_organized_ (false), user_filter_value_ (std::numeric_limits<float>::quiet_NaN ())
      {
        filter_name_ = "PassThrough";
      };

      /** \brief Set whether the filtered points should be kept and set to the
        * value given through \a setUserFilterValue (default: NaN), or removed
        * from the PointCloud, thus potentially breaking its organized
        * structure. By default, points are removed.
        *
        * \param val set to true whether the filtered points should be kept and
        * set to a given user value (default: NaN)
        */
      inline void setKeepOrganized (bool val) { keep_organized_ = val; }
      
      inline bool getKeepOrganized () { return (keep_organized_); }

      /** \brief Provide a value that the filtered points should be set to
        * instead of removing them.  Used in conjunction with \a
        * setKeepOrganized ().
        * \param val the user given value that the filtered point dimensions should be set to
        */
      inline void setUserFilterValue (float val) { user_filter_value_ = val; }

    protected:
      void applyFilter (PointCloud2 &output);

    private:
      /** \brief Keep the structure of the data organized, by setting the
       * filtered points to the a user given value (NaN by default). */
      bool keep_organized_;

      /** \brief User given value to be set to any filtered point. Casted to
       * the correct field type. */
      float user_filter_value_;

  };
}

#endif  //#ifndef PCL_FILTERS_PASSTHROUGH_H_
