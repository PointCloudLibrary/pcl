/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *  $Id$
 */

#pragma once

#include <pcl/pcl_base.h>
#include <list>

namespace pcl
{
  /** \brief Class for extracting interest regions from unstructured point clouds, based on a multi scale
    * statistical approach.
    * Please refer to the following publications for more details:
    *    Ranjith Unnikrishnan and Martial Hebert
    *    Multi-Scale Interest Regions from Unorganized Point Clouds
    *    Workshop on Search in 3D (S3D), IEEE Conf. on Computer Vision and Pattern Recognition (CVPR)
    *    June, 2008
    *
    *    Statistical Approaches to Multi-scale Point Cloud Processing
    *    Ranjith Unnikrishnan
    *    PhD Thesis
    *    The Robotics Institute Carnegie Mellon University
    *    May, 2008
    *
    * \author Alexandru-Eugen Ichim
    */
  template <typename PointT>
  class StatisticalMultiscaleInterestRegionExtraction : public PCLBase<PointT>
  {
    public:
      using IndicesPtr = shared_ptr<pcl::Indices >;
      using Ptr = shared_ptr<StatisticalMultiscaleInterestRegionExtraction<PointT> >;
      using ConstPtr = shared_ptr<const StatisticalMultiscaleInterestRegionExtraction<PointT> >;


      /** \brief Empty constructor */
      StatisticalMultiscaleInterestRegionExtraction ()
      {};

      /** \brief Method that generates the underlying nearest neighbor graph based on the
       * input point cloud
       */
      void
      generateCloudGraph ();

      /** \brief The method to be called in order to run the algorithm and produce the resulting
       * set of regions of interest
       */
      void
      computeRegionsOfInterest (std::list<IndicesPtr>& rois);

      /** \brief Method for setting the scale parameters for the algorithm
       * \param scale_values vector of scales to determine the size of each scaling step
       */
      inline void
      setScalesVector (std::vector<float> &scale_values) { scale_values_ = scale_values; }

      /** \brief Method for getting the scale parameters vector */
      inline std::vector<float>
      getScalesVector () { return scale_values_; }


    private:
      /** \brief Checks if all the necessary input was given and the computations can successfully start */
      bool
      initCompute ();

      void
      geodesicFixedRadiusSearch (std::size_t &query_index,
                                 float &radius,
                                 std::vector<int> &result_indices);

      void
      computeF ();

      void
      extractExtrema (std::list<IndicesPtr>& rois);

      using PCLBase<PointT>::initCompute;
      using PCLBase<PointT>::input_;
      std::vector<float> scale_values_;
      std::vector<std::vector<float> > geodesic_distances_;
      std::vector<std::vector<float> > F_scales_;
  };
}


#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/statistical_multiscale_interest_region_extraction.hpp>
#endif
