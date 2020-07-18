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

#pragma once

#include <pcl/keypoints/keypoint.h>

namespace pcl
{
  /** \brief
   * Based on the paper:
   *    Xinju Li and Igor Guskov
   *    Multi-scale features for approximate alignment of point-based surfaces
   *    Proceedings of the third Eurographics symposium on Geometry processing
   *    July 2005, Vienna, Austria
   *
   * \author Alexandru-Eugen Ichim
   */
  template <typename PointT, typename PointNT>
  class SmoothedSurfacesKeypoint : public Keypoint <PointT, PointT>
  {
    public:
      using Ptr = shared_ptr<SmoothedSurfacesKeypoint<PointT, PointNT> >;
      using ConstPtr = shared_ptr<const SmoothedSurfacesKeypoint<PointT, PointNT> >;

      using PCLBase<PointT>::input_;
      using Keypoint<PointT, PointT>::name_;
      using Keypoint<PointT, PointT>::tree_;
      using Keypoint<PointT, PointT>::keypoints_indices_;
      using Keypoint<PointT, PointT>::initCompute;

      using PointCloudT = pcl::PointCloud<PointT>;
      using PointCloudTConstPtr = typename PointCloudT::ConstPtr;
      using PointCloudNT = pcl::PointCloud<PointNT>;
      using PointCloudNTConstPtr = typename PointCloudNT::ConstPtr;
      using PointCloudTPtr = typename PointCloudT::Ptr;
      using KdTreePtr = typename Keypoint<PointT, PointT>::KdTreePtr;

      SmoothedSurfacesKeypoint ()
        : Keypoint<PointT, PointT> (),
          neighborhood_constant_ (0.5f),
          clouds_ (),
          cloud_normals_ (),
          cloud_trees_ (),
          normals_ (),
          input_scale_ (0.0f),
          input_index_ ()
      {
        name_ = "SmoothedSurfacesKeypoint";

        // hack to pass the initCompute () check of Keypoint - although it is never used in SmoothedSurfacesKeypoint
        Keypoint<PointT, PointT>::search_radius_ = 0.1;
      }

      void
      addSmoothedPointCloud (const PointCloudTConstPtr &cloud,
                             const PointCloudNTConstPtr &normals,
                             KdTreePtr &kdtree,
                             float &scale);


      void
      resetClouds ();

      inline void
      setNeighborhoodConstant (float neighborhood_constant) { neighborhood_constant_ = neighborhood_constant; }

      inline float
      getNeighborhoodConstant () { return neighborhood_constant_; }

      inline void
      setInputNormals (const PointCloudNTConstPtr &normals) { normals_ = normals; }

      inline void
      setInputScale (float input_scale) { input_scale_ = input_scale; }

      void
      detectKeypoints (PointCloudT &output) override;

    protected:
      bool
      initCompute () override;

    private:
      float neighborhood_constant_;
      std::vector<PointCloudTConstPtr> clouds_;
      std::vector<PointCloudNTConstPtr> cloud_normals_;
      std::vector<KdTreePtr> cloud_trees_;
      PointCloudNTConstPtr normals_;
      std::vector<std::pair<float, std::size_t> > scales_;
      float input_scale_;
      std::size_t input_index_;

      static bool
      compareScalesFunction (const std::pair<float, std::size_t> &a,
                             const std::pair<float, std::size_t> &b) { return a.first < b.first; }
  };
}
