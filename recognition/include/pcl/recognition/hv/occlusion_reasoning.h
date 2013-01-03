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
 */

#ifndef PCL_RECOGNITION_OCCLUSION_REASONING_H_
#define PCL_RECOGNITION_OCCLUSION_REASONING_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

namespace pcl
{

  namespace occlusion_reasoning
  {
    /**
     * \brief Class to reason about occlusions
     * \author Aitor Aldoma
     */

    template<typename ModelT, typename SceneT>
      class ZBuffering
      {
      private:
        float f_;
        int cx_, cy_;
        float * depth_;

      public:

        ZBuffering ();
        ZBuffering (int resx, int resy, float f);
        ~ZBuffering ();
        void
        computeDepthMap (typename pcl::PointCloud<SceneT>::ConstPtr & scene, bool compute_focal = false, bool smooth = false, int wsize = 3);
        void
        filter (typename pcl::PointCloud<ModelT>::ConstPtr & model, typename pcl::PointCloud<ModelT>::Ptr & filtered, float thres = 0.01);
        void filter (typename pcl::PointCloud<ModelT>::ConstPtr & model, std::vector<int> & indices, float thres = 0.01);
      };

    template<typename ModelT, typename SceneT> typename pcl::PointCloud<ModelT>::Ptr
    filter (typename pcl::PointCloud<SceneT>::ConstPtr & organized_cloud, typename pcl::PointCloud<ModelT>::ConstPtr & to_be_filtered, float f,
            float threshold)
    {
      float cx = (static_cast<float> (organized_cloud->width) / 2.f - 0.5f);
      float cy = (static_cast<float> (organized_cloud->height) / 2.f - 0.5f);
      typename pcl::PointCloud<ModelT>::Ptr filtered (new pcl::PointCloud<ModelT> ());

      std::vector<int> indices_to_keep;
      indices_to_keep.resize (to_be_filtered->points.size ());

      int keep = 0;
      for (size_t i = 0; i < to_be_filtered->points.size (); i++)
      {
        float x = to_be_filtered->points[i].x;
        float y = to_be_filtered->points[i].y;
        float z = to_be_filtered->points[i].z;
        int u = static_cast<int> (f * x / z + cx);
        int v = static_cast<int> (f * y / z + cy);

        //Not out of bounds
        if ((u >= static_cast<int> (organized_cloud->width)) || (v >= static_cast<int> (organized_cloud->height)) || (u < 0) || (v < 0))
          continue;

        //Check for invalid depth
        if (!pcl_isfinite (organized_cloud->at (u, v).x) || !pcl_isfinite (organized_cloud->at (u, v).y)
            || !pcl_isfinite (organized_cloud->at (u, v).z))
          continue;

        float z_oc = organized_cloud->at (u, v).z;

        //Check if point depth (distance to camera) is greater than the (u,v)
        if ((z - z_oc) > threshold)
          continue;

        indices_to_keep[keep] = static_cast<int> (i);
        keep++;
      }

      indices_to_keep.resize (keep);
      pcl::copyPointCloud (*to_be_filtered, indices_to_keep, *filtered);
      return filtered;
    }

    template<typename ModelT, typename SceneT> typename pcl::PointCloud<ModelT>::Ptr
    filter (typename pcl::PointCloud<SceneT>::Ptr & organized_cloud, typename pcl::PointCloud<ModelT>::Ptr & to_be_filtered, float f,
            float threshold, bool check_invalid_depth = true)
    {
      float cx = (static_cast<float> (organized_cloud->width) / 2.f - 0.5f);
      float cy = (static_cast<float> (organized_cloud->height) / 2.f - 0.5f);
      typename pcl::PointCloud<ModelT>::Ptr filtered (new pcl::PointCloud<ModelT> ());

      std::vector<int> indices_to_keep;
      indices_to_keep.resize (to_be_filtered->points.size ());

      int keep = 0;
      for (size_t i = 0; i < to_be_filtered->points.size (); i++)
      {
        float x = to_be_filtered->points[i].x;
        float y = to_be_filtered->points[i].y;
        float z = to_be_filtered->points[i].z;
        int u = static_cast<int> (f * x / z + cx);
        int v = static_cast<int> (f * y / z + cy);

        //Not out of bounds
        if ((u >= static_cast<int> (organized_cloud->width)) || (v >= static_cast<int> (organized_cloud->height)) || (u < 0) || (v < 0))
          continue;

        //Check for invalid depth
        if (check_invalid_depth)
        {
          if (!pcl_isfinite (organized_cloud->at (u, v).x) || !pcl_isfinite (organized_cloud->at (u, v).y)
              || !pcl_isfinite (organized_cloud->at (u, v).z))
            continue;
        }

        float z_oc = organized_cloud->at (u, v).z;

        //Check if point depth (distance to camera) is greater than the (u,v)
        if ((z - z_oc) > threshold)
          continue;

        indices_to_keep[keep] = static_cast<int> (i);
        keep++;
      }

      indices_to_keep.resize (keep);
      pcl::copyPointCloud (*to_be_filtered, indices_to_keep, *filtered);
      return filtered;
    }

    template<typename ModelT, typename SceneT> typename pcl::PointCloud<ModelT>::Ptr
    getOccludedCloud (typename pcl::PointCloud<SceneT>::Ptr & organized_cloud, typename pcl::PointCloud<ModelT>::Ptr & to_be_filtered, float f,
                      float threshold, bool check_invalid_depth = true)
    {
      float cx = (static_cast<float> (organized_cloud->width) / 2.f - 0.5f);
      float cy = (static_cast<float> (organized_cloud->height) / 2.f - 0.5f);
      typename pcl::PointCloud<ModelT>::Ptr filtered (new pcl::PointCloud<ModelT> ());

      std::vector<int> indices_to_keep;
      indices_to_keep.resize (to_be_filtered->points.size ());

      int keep = 0;
      for (size_t i = 0; i < to_be_filtered->points.size (); i++)
      {
        float x = to_be_filtered->points[i].x;
        float y = to_be_filtered->points[i].y;
        float z = to_be_filtered->points[i].z;
        int u = static_cast<int> (f * x / z + cx);
        int v = static_cast<int> (f * y / z + cy);

        //Out of bounds
        if ((u >= static_cast<int> (organized_cloud->width)) || (v >= static_cast<int> (organized_cloud->height)) || (u < 0) || (v < 0))
          continue;

        //Check for invalid depth
        if (check_invalid_depth)
        {
          if (!pcl_isfinite (organized_cloud->at (u, v).x) || !pcl_isfinite (organized_cloud->at (u, v).y)
              || !pcl_isfinite (organized_cloud->at (u, v).z))
            continue;
        }

        float z_oc = organized_cloud->at (u, v).z;

        //Check if point depth (distance to camera) is greater than the (u,v)
        if ((z - z_oc) > threshold)
        {
          indices_to_keep[keep] = static_cast<int> (i);
          keep++;
        }
      }

      indices_to_keep.resize (keep);
      pcl::copyPointCloud (*to_be_filtered, indices_to_keep, *filtered);
      return filtered;
    }
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/recognition/impl/hv/occlusion_reasoning.hpp>
#endif

#endif /* PCL_RECOGNITION_OCCLUSION_REASONING_H_ */
