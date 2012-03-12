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

#ifndef PCL_FILTER_INDICES_H_
#define PCL_FILTER_INDICES_H_

#include <pcl/filters/filter.h>

namespace pcl
{
  /** \brief Removes points with x, y, or z equal to NaN
    * \param cloud_in the input point cloud
    * \param index the mapping (ordered): cloud_out.points[i] = cloud_in.points[index[i]]
    * \note The density of the point cloud is lost.
    * \note Can be called with cloud_in == cloud_out
    * \ingroup filters
    */
  template<typename PointT> void
  removeNaNFromPointCloud (const pcl::PointCloud<PointT> &cloud_in, std::vector<int> &index);

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b Filter represents the base filter class. Some generic 3D operations that are applicable to all filters
    * are defined here as static methods.
    * \author Justin Rosen
    * \ingroup filters
    */
  template<typename PointT>
  class FilterIndices : public Filter<PointT>
  {

    using Filter<PointT>::initCompute;
    using Filter<PointT>::deinitCompute;

    public:

      typedef pcl::PointCloud<PointT> PointCloud;

      virtual void
      filter (PointCloud &output)
      {
        pcl::Filter<PointT>::filter (output);
      }

      /** \brief Calls the filtering method and returns the filtered point cloud
        * indices.
        * \param indices the resultant filtered point cloud indices
        */
      inline void
      filter (std::vector<int> &indices)
      {
        if (!initCompute ())
          return;

        // Apply the actual filter
        applyFilter (indices);

        deinitCompute ();
      }

    protected:

      /** \brief Abstract filter method for point cloud indices.
        *
        * The implementation needs to set indices.
        */
      virtual void
      applyFilter (std::vector<int> &indices) = 0;

  };

  /** \brief @b FilterIndices represents the base filter with indices class.
    * Some generic 3D operations that are applicable to all filters are defined
    * here as static methods.
    * \author Justin Rosen
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS FilterIndices<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {

    public:

      typedef sensor_msgs::PointCloud2 PointCloud2;

      virtual void
      filter (PointCloud2 &output)
      {
        pcl::Filter<PointCloud2>::filter (output);
      }

      /** \brief Calls the filtering method and returns the filtered dataset in output.
        * \param indices the resultant indices
        */
      void
      filter (std::vector<int> &indices);

    protected:

      /** \brief Abstract filter method for point cloud indices.
        *
        * The implementation needs to set indices.
        */
      virtual void
      applyFilter (std::vector<int> &indices) = 0;

  };
}

#endif  //#ifndef PCL_FILTER_INDICES_H_
