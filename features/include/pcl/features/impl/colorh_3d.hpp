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

#ifndef PCL_FEATURES_IMPL_COLORH_3D_H_
#define PCL_FEATURES_IMPL_COLORH_3D_H_

#include <algorithm>
#include <pcl/features/colorh_3d.h>
#include <pcl/common/fft/kiss_fftr.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> void
pcl::ColorHEstimation3D<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  //Set the bin size and bin intervals
  int bin_dims = 4;
  double color_3d[bin_dims][bin_dims][bin_dims];
  double bin_interval = 256.0 / bin_dims;
  double bin_interval_h = 360.0 / bin_dims;
  double bin_interval_s = 1.0 / bin_dims;
  double bin_interval_v = 1.0 / bin_dims;
  
  //Initialize the temp histogram matrix with zeros
  double R=0, G=0, B=0;
  for (int i = 0; i < bin_dims; i++)
    for (int j = 0; j < bin_dims; j++)
      for (int k = 0; k < bin_dims; k++)
        color_3d[i][j][k] = 0.0;
  
  //Keep binning the histogram based on the color of the points in cloud
  for (size_t i = 0; i < input_->points.size (); i++)
  {
    R = input_->points[i].r;
    G = input_->points[i].g;
    B = input_->points[i].b;
    int ind_x, ind_y, ind_z;
    //Checking if the flag is set to rgb histogram
    if (isrgb_)
    {
      ind_x = R/bin_interval;
      ind_y = G/bin_interval;
      ind_z = B/bin_interval;
      color_3d[ind_x][ind_y][ind_z]++;
    }
    //Checking if the flag is set to yuv histogram
    else if (isyuv_)
    {
      double Y, U, V;
      Y = R * 0.299000 + G * 0.587000 + B * 0.114000;
      U = - R * 0.168736 - G * 0.331264 + B * 0.500000 + 128;
      V = R * 0.500000 - G * 0.418688 - B * 0.081312 + 128;
      ind_x = Y/bin_interval;
      ind_y = U/bin_interval;
      ind_z = V/bin_interval;
      color_3d[ind_x][ind_y][ind_z]++;
    }
    //If the flag is set to hsv histogram
    else
    {
      double r, g, b, h, s, v, d;
      double min_, max_;
      r = (double)R/255.0;
      g = (double)G/255.0;
      b = (double)B/255.0;
      min_ = std::min (r, std::min (g, b));
      max_ = std::max (r, std::max (g, b));
      if (max_ == min_)
      {
        v = max_;
        h = 0;
        s = 0;
      }
      else
      {
        d = (r==min_) ? g-b : ( (b==min_) ? r-g : b-r);
        h = (r==min_) ? 3 : ( (b==min_) ? 1 : 5);
        h = 60 * (h - d/ (max_ - min_));
        s = (max_ - min_)/max_;
        v = max_;
      }
      ind_x = h/bin_interval_h;
      ind_y = s/bin_interval_s;
      ind_z = v/bin_interval_v;
      color_3d[ind_x][ind_y][ind_z]++;
    }
  }
  //Initialize the size of the output cloud
  output.points.resize (1);
  output.width = output.height = 1;
  int a = 0;

  //Copy the histogram to the output point cloud
  for (int i = 0; i < bin_dims; i++)
    for (int j = 0; j < bin_dims; j++)
      for (int k = 0; k < bin_dims; k++)
      {
        output.points[0].histogram[a] = color_3d[i][j][k]/input_->points.size ();
        a++;
      }
}

#define PCL_INSTANTIATE_ColorHEstimation3D(T,OutT) template class PCL_EXPORTS pcl::ColorHEstimation3D<T,OutT>;

#endif    // PCL_FEATURES_IMPL_COLORH_3D_H_
