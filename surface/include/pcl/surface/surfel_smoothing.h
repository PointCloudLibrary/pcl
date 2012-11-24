/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 */

#ifndef PCL_SURFEL_SMOOTHING_H_
#define PCL_SURFEL_SMOOTHING_H_

#include <pcl/pcl_base.h>
#include <pcl/search/pcl_search.h>

namespace pcl
{
  template <typename PointT, typename PointNT>
  class SurfelSmoothing : public PCLBase<PointT>
  {
    using PCLBase<PointT>::input_;
    using PCLBase<PointT>::initCompute;

    public:
      typedef boost::shared_ptr<SurfelSmoothing<PointT, PointNT> > Ptr;
      typedef boost::shared_ptr<const SurfelSmoothing<PointT, PointNT> > ConstPtr;

      typedef pcl::PointCloud<PointT> PointCloudIn;
      typedef typename pcl::PointCloud<PointT>::Ptr PointCloudInPtr;
      typedef pcl::PointCloud<PointNT> NormalCloud;
      typedef typename pcl::PointCloud<PointNT>::Ptr NormalCloudPtr;
      typedef pcl::search::Search<PointT> CloudKdTree;
      typedef typename pcl::search::Search<PointT>::Ptr CloudKdTreePtr;

      SurfelSmoothing (float a_scale = 0.01)
        : PCLBase<PointT> ()
        , scale_ (a_scale)
        , scale_squared_ (a_scale * a_scale)
        , normals_ ()
        , interm_cloud_ ()
        , interm_normals_ ()
        , tree_ ()
      {
      }

      void
      setInputNormals (NormalCloudPtr &a_normals) { normals_ = a_normals; };

      void
      setSearchMethod (const CloudKdTreePtr &a_tree) { tree_ = a_tree; };

      bool
      initCompute ();

      float
      smoothCloudIteration (PointCloudInPtr &output_positions,
                            NormalCloudPtr &output_normals);

      void
      computeSmoothedCloud (PointCloudInPtr &output_positions,
                            NormalCloudPtr &output_normals);


      void
      smoothPoint (size_t &point_index,
                   PointT &output_point,
                   PointNT &output_normal);

      void
      extractSalientFeaturesBetweenScales (PointCloudInPtr &cloud2,
                                           NormalCloudPtr &cloud2_normals,
                                           boost::shared_ptr<std::vector<int> > &output_features);

    private:
      float scale_, scale_squared_;
      NormalCloudPtr normals_;

      PointCloudInPtr interm_cloud_;
      NormalCloudPtr interm_normals_;

      CloudKdTreePtr tree_;

  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/surfel_smoothing.hpp>
#endif

#endif    // PCL_SURFEL_SMOOTHING_H_
