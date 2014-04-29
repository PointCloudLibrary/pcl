/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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

#ifndef PCL_FILTERS_LOCAL_MAXIMUM_H_
#define PCL_FILTERS_LOCAL_MAXIMUM_H_

#include <pcl/filters/filter_indices.h>
#include <pcl/search/pcl_search.h>

namespace pcl
{
  /** \brief LocalMaximum downsamples the cloud, by eliminating points that are locally maximal.
    *
    * The LocalMaximum class analyzes each point and removes those that are
    * found to be locally maximal with respect to their neighbors (found via
    * radius search). The comparison is made in the z dimension only at this
    * time.
    *
    * \author Bradley J Chambers
    * \ingroup filters
    */
  template <typename PointT>
  class LocalMaximum: public FilterIndices<PointT>
  {
    protected:
      typedef typename FilterIndices<PointT>::PointCloud PointCloud;
      typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;

    public:
      /** \brief Empty constructor. */
      LocalMaximum (bool extract_removed_indices = false) :
        FilterIndices<PointT>::FilterIndices (extract_removed_indices),
        searcher_ (),
        radius_ (1)
      {
        filter_name_ = "LocalMaximum";
      }

      /** \brief Set the radius to use to determine if a point is the local max.
        * \param[in] radius The radius to use to determine if a point is the local max.
        */
      inline void
      setRadius (float radius) { radius_ = radius; }

      /** \brief Get the radius to use to determine if a point is the local max.
        * \return The radius to use to determine if a point is the local max.
        */
      inline float
      getRadius () const { return (radius_); }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Downsample a Point Cloud by eliminating points that are locally maximal in z
        * \param[out] output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilter (std::vector<int> &indices)
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilterIndices (std::vector<int> &indices);

    private:
      /** \brief A pointer to the spatial search object. */
      SearcherPtr searcher_;

      /** \brief The radius to use to determine if a point is the local max. */
      float radius_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/local_maximum.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_LOCAL_MAXIMUM_H_

