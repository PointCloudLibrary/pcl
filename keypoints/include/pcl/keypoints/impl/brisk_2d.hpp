/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (C) 2011, The Autonomous Systems Lab (ASL), ETH Zurich, 
 *                      Stefan Leutenegger, Simon Lynen and Margarita Chli.
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
 */

#ifndef PCL_KEYPOINTS_BRISK_KEYPOINT_2D_IMPL_H_
#define PCL_KEYPOINTS_BRISK_KEYPOINT_2D_IMPL_H_

#include <pcl/common/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> bool
pcl::BriskKeypoint2D<PointInT, PointOutT, IntensityT>::initCompute ()
{
  if (!pcl::Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed.!\n", name_.c_str ());
    return (false);
  }

  if (!input_->isOrganized ())
  {    
    PCL_ERROR ("[pcl::%s::initCompute] %s doesn't support non organized clouds!\n", name_.c_str ());
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename IntensityT> void
pcl::BriskKeypoint2D<PointInT, PointOutT, IntensityT>::detectKeypoints (PointCloudOut &output)
{
  // image size
  const int width = int (input_->width);
  const int height = int (input_->height);

  // destination for intensity data; will be forwarded to BRISK
  std::vector<unsigned char> image_data (width*height);

  for (size_t i = 0; i < image_data.size (); ++i)
    image_data[i] = static_cast<unsigned char> (intensity_ ((*input_)[i]));

  pcl::keypoints::brisk::ScaleSpace brisk_scale_space (octaves_);
  brisk_scale_space.constructPyramid (image_data, width, height);
  pcl::PointCloud<pcl::PointWithScale> output_temp;
  brisk_scale_space.getKeypoints (threshold_, output_temp.points);
  pcl::copyPointCloud<pcl::PointWithScale, PointOutT> (output_temp, output);

  // we do not change the denseness
  output.width = int (output.points.size ());
  output.height = 1;
  output.is_dense = false;      // set to false to be sure

  // 2nd pass to remove the invalid set of 3D keypoints
  if (remove_invalid_3D_keypoints_)
  {
    PointCloudOut output_clean;
    for (size_t i = 0; i < output.size (); ++i)
    {
      PointOutT pt;
      // Interpolate its position in 3D, as the "u" and "v" are subpixel accurate
      bilinearInterpolation (input_, output[i].x, output[i].y, pt);

      // Check if the point is finite
      if (pcl::isFinite (pt))
        output_clean.push_back (output[i]);
    }
    output = output_clean;
    output.is_dense = true;      // set to true as there's no keypoint at an invalid XYZ
  }
}

#endif 
