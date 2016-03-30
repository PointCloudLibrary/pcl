/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010-2011, Willow Garage, Inc.
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
*
*/

#ifndef PCL_FEATURES_IMPL_LRF_UTILS_H_
#define PCL_FEATURES_IMPL_LRF_UTILS_H_


#include <pcl/features/lrf_utils.h>



template<typename PointNT> bool
  pcl::normalDisambiguation (
  pcl::PointCloud<PointNT> const &normal_cloud,
  std::vector<int> const &normal_indices,
  Eigen::Vector3f &normal)
{
  Eigen::Vector3f normal_mean;
  normal_mean.setZero ();

  bool at_least_one_valid_point = false;
  for (size_t i = 0; i < normal_indices.size (); ++i)
  {
    const PointNT& curPt = normal_cloud[normal_indices[i]];

    if(pcl::isFinite(curPt))
    {
      normal_mean += curPt.getNormalVector3fMap ();
      at_least_one_valid_point = true;
    }
  }

  if(!at_least_one_valid_point)
    return false;

  normal_mean.normalize ();

  if (normal.dot (normal_mean) < 0)
  {
    normal = -normal;
  }

  return true;
}


#define PCL_INSTANTIATE_normalDisambiguation(NT) template PCL_EXPORTS bool pcl::normalDisambiguation<NT>( pcl::PointCloud<NT> const &normal_cloud, std::vector<int> const &normal_indices, Eigen::Vector3f &normal);


#endif
