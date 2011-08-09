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

#ifndef PCL_KEYPOINTS_IMPL_SMOOTHEDSURFACESKEYPOINT_H_
#define PCL_KEYPOINTS_IMPL_SMOOTHEDSURFACESKEYPOINT_H_

#include "pcl/keypoints/smoothed_surfaces_keypoint.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointOut> void
pcl::SmoothedSurfacesKeypoint<PointT, PointNT, PointOut>::addSmoothedPointCloud (SmoothedSurfacesKeypoint<PointT, PointNT, PointOut>::PointCloudTPtr &cloud,
                                                                                 SmoothedSurfacesKeypoint<PointT, PointNT, PointOut>::PointCloudNTPtr &normals,
                                                                                 float &scale)
{
  clouds_.push_back (cloud);
  normals_.push_back (normals);
  scales_.push_back (scale);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointOut> void
pcl::SmoothedSurfacesKeypoint<PointT, PointNT, PointOut>::resetClouds ()
{
  clouds_.clear ();
  normals_.clear ();
  scales_.clear ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointOut> void
pcl::SmoothedSurfacesKeypoint<PointT, PointNT, PointOut>::detectKeypoints (PointCloudOut &output)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT, typename PointOut> void
pcl::SmoothedSurfacesKeypoint<PointT, PointNT, PointOut>::initCompute ()
{
  PCL_INFO ("SmoothedSurfacesKeypoint initCompute () called\n");
}


#define PCL_INSTANTIATE_SmoothedSurfacesKeypoint(T,NT,OutT) template class PCL_EXPORTS pcl::SmoothedSurfacesKeypoint<T,NT,OutT>;


#endif /* PCL_KEYPOINTS_IMPL_SMOOTHEDSURFACESKEYPOINT_H_ */
