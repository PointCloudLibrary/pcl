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

#ifndef PCL_SMOOTHEDSURFACESKEYPOINT_H_
#define PCL_SMOOTHEDSURFACESKEYPOINT_H_

#include "pcl/keypoints/keypoint.h"

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
  template <typename PointT, typename PointNT, typename PointOut>
  class SmoothedSurfacesKeypoint : public Keypoint <PointT, PointOut>
  {
    public:
      using Keypoint<PointT, PointOut>::name_;

      typedef pcl::PointCloud<PointT> PointCloudT;
      typedef typename PointCloudT::Ptr PointCloudTPtr;
      typedef pcl::PointCloud<PointNT> PointCloudNT;
      typedef typename PointCloudNT::Ptr PointCloudNTPtr;
      typedef pcl::PointCloud<PointOut> PointCloudOut;
      typedef typename PointCloudOut::Ptr PointCloudOutPtr;

      SmoothedSurfacesKeypoint ()
        : Keypoint<PointT, PointOut> (),
          neighborhood_constant_ (0.5f)
      {
        name_ = "SmoothedSurfacesKeypoint";
      }

    void
    addSmoothedPointCloud (PointCloudTPtr &cloud, PointCloudNTPtr &normals, float &scale);

    void
    resetClouds ();

    inline void
    setNeighborhoodConstant (float neighborhood_constant) { neighborhood_constant_ = neighborhood_constant; }

    inline float
    getNeighborhoodConstant () { return neighborhood_constant_; }

    protected:
      void
      detectKeypoints (PointCloudOut &output);

      void
      initCompute ();

    private:
      float neighborhood_constant_;
      std::vector<PointCloudTPtr> clouds_;
      std::vector<PointCloudNTPtr> normals_;
      std::vector<float> scales_;
  };
}

#endif /* PCL_SMOOTHEDSURFACESKEYPOINT_H_ */
