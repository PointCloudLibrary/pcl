/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#pragma once

#include <pcl_cuda/pcl_cuda_base.h>
#include <limits>

namespace pcl_cuda
{
  /** \brief Removes points with x, y, or z equal to NaN
    * \param cloud_in the input point cloud
    * \param cloud_out the input point cloud
    * \param index the mapping (ordered): cloud_out[i] = cloud_in[index[i]]
    * \note The density of the point cloud is lost.
    * \note Can be called with cloud_in == cloud_out
    */
//  template <typename PointT> void removeNaNFromPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, std::vector<int> &index);

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b Filter represents the base filter class. Some generic 3D
    * operations that are applicable to all filters are defined here as static
    * methods.
    */
  template <typename CloudT>
  class Filter : public PCLCUDABase<CloudT>
  {
    using PCLCUDABase<CloudT>::initCompute;
    using PCLCUDABase<CloudT>::deinitCompute;

    public:
      using PCLCUDABase<CloudT>::input_;

      using PointCloud = typename PCLCUDABase<CloudT>::PointCloud;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      /** \brief Empty constructor. */
      Filter () : filter_field_name_ (""), 
                  filter_limit_min_ (std::numeric_limits<float>::lowest()),
                  filter_limit_max_ (std::numeric_limits<float>::max()),
                  filter_limit_negative_ (false)
      {};

      /** \brief Provide the name of the field to be used for filtering data.
        * In conjunction with  \a setFilterLimits, points having values outside
        * this interval will be discarded.
        * \param field_name the name of the field that contains values used for filtering
        */
      inline void 
      setFilterFieldName (const std::string &field_name) { filter_field_name_ = field_name; }

      /** \brief Get the name of the field used for filtering. */
      inline std::string const 
      getFilterFieldName () { return (filter_field_name_); }

      /** \brief Set the field filter limits. All points having field values
        * outside this interval will be discarded.  
        * \param limit_min the minimum allowed field value
        * \param limit_max the maximum allowed field value
        */
      inline void
      setFilterLimits (const double &limit_min, const double &limit_max)
      {
        filter_limit_min_ = limit_min;
        filter_limit_max_ = limit_max;
      }

      /** \brief Get the field filter limits (min/max) set by the user. 
        * The default values are std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max().
        * \param limit_min the minimum limit
        * \param limit_max the maximum limit
        */
      inline void
      getFilterLimits (double &limit_min, double &limit_max)
      {
        limit_min = filter_limit_min_;
        limit_max = filter_limit_max_;
      }

      /** \brief Set to true if we want to return the data outside the interval
        * specified by setFilterLimits (min, max).  Default: false.
        * \param limit_negative return data inside the interval (false) or outside (true)
        */
      inline void 
      setFilterLimitsNegative (const bool limit_negative) 
      { 
        filter_limit_negative_ = limit_negative; 
      }

      /** \brief Get whether the data outside the interval (min/max) is to be
        * returned (true) or inside (false). 
        * \param limit_negative the limit_negative flag
        */
      PCL_DEPRECATED(1, 16, "use bool getFilterLimitsNegative() instead")
      inline void 
      getFilterLimitsNegative (bool &limit_negative) { limit_negative = filter_limit_negative_; }

      /** \brief Get whether the data outside the interval (min/max) is to be
        * returned (true) or inside (false). 
        * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline bool 
      getFilterLimitsNegative () { return (filter_limit_negative_); }

      /** \brief Calls the filtering method and returns the filtered dataset on the device
        * \param output the resultant filtered point cloud dataset on the device
        */
      inline void
      filter (PointCloud &output)
      {
        if (!initCompute ()) return;

        // Copy header at a minimum
        //output.header = input_->header;
        //output.sensor_origin_ = input_->sensor_origin_;
        //output.sensor_orientation_ = input_->sensor_orientation_;

        // Apply the actual filter
        applyFilter (output);

        deinitCompute ();
      }

    protected:
      /** \brief The filter name. */
      std::string filter_name_;

      /** \brief The desired user filter field name. */
      std::string filter_field_name_;

      /** \brief The minimum allowed filter value a point will be considered from. */
      double filter_limit_min_;

      /** \brief The maximum allowed filter value a point will be considered from. */
      double filter_limit_max_;

      /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
      bool filter_limit_negative_;

      /** \brief Abstract filter method. 
        * 
        * The implementation needs to set output.{points, width, height, is_dense}.
        */
      virtual void 
      applyFilter (PointCloud &output) = 0;

      /** \brief Get a string representation of the name of this class. */
      inline const std::string& 
      getClassName () const { return (filter_name_); }
  };
}
