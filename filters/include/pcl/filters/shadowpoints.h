/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_FILTERS_SHADOW_POINTS_FILTER_H_
#define PCL_FILTERS_SHADOW_POINTS_FILTER_H_

#include <pcl/filters/filter_indices.h>
#include <time.h>
#include <limits.h>

namespace pcl
{
  /** \brief @b ShadowPoints removes the ghost points appearing on edge discontinuties
   *
   *  \author Aravindhan K Krishnan. This code is ported from libpointmatcher (https://github.com/ethz-asl/libpointmatcher)
   * \ingroup filters
   */
  template<typename PointT, typename NormalT>
  class ShadowPoints : public FilterIndices<PointT>
  {
    using FilterIndices<PointT>::filter_name_;
    using FilterIndices<PointT>::getClassName;
    using FilterIndices<PointT>::indices_;
    using FilterIndices<PointT>::input_;
    using FilterIndices<PointT>::removed_indices_;
    using FilterIndices<PointT>::extract_removed_indices_;
    using FilterIndices<PointT>::negative_;
    using FilterIndices<PointT>::user_filter_value_;
    using FilterIndices<PointT>::keep_organized_;

    typedef typename FilterIndices<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef typename pcl::PointCloud<NormalT>::Ptr NormalsPtr;

    public:

      typedef boost::shared_ptr< ShadowPoints<PointT, NormalT> > Ptr;
      typedef boost::shared_ptr< const ShadowPoints<PointT, NormalT> > ConstPtr;

      /** \brief Empty constructor. */
      ShadowPoints (bool extract_removed_indices = false) : 
        FilterIndices<PointT> (extract_removed_indices),
        input_normals_ (), 
        threshold_ (0.1f) 
      {
        filter_name_ = "ShadowPoints";
      }

      /** \brief Set the normals computed on the input point cloud
        * \param[in] normals the normals computed for the input cloud
        */
      inline void 
      setNormals (const NormalsPtr &normals) { input_normals_ = normals; }

      /** \brief Get the normals computed on the input point cloud */
      inline NormalsPtr
      getNormals () const { return (input_normals_); }

      /** \brief Set the threshold for shadow points rejection
        * \param[in] threshold the threshold
        */
      inline void
      setThreshold (float threshold) { threshold_ = threshold; }

      /** \brief Get the threshold for shadow points rejection */
      inline float 
      getThreshold () const { return threshold_; }

    protected:
     
      /** \brief The normals computed at each point in the input cloud */
      NormalsPtr input_normals_; 

      /** \brief Sample of point indices into a separate PointCloud
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Sample of point indices
        * \param[out] indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);

    private:

      /** \brief Threshold for shadow point rejection
        */
      float threshold_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/shadowpoints.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_SHADOW_POINTS_FILTER_H_
