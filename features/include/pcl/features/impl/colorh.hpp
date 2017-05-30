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

#ifndef PCL_FEATURES_IMPL_COLORH_H_
#define PCL_FEATURES_IMPL_COLORH_H_

#include <algorithm>
#include <pcl/features/colorh.h>
#include <pcl/common/fft/kiss_fftr.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT> void
pcl::ColorHEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  //Set the bin size and bin_intervals 
  int nbins = nbins_;
  double bin_interval = 3*256.0 / nbins;
  double bin_interval_h = 3*360.0 / nbins;
  double bin_interval_s = 3*1.0 / nbins;
  double bin_interval_v = 3*1.0 / nbins;
  
  //Initialize a temp histogram with zeros
  std::vector<double> color;
  color.resize(nbins_);
  for (int i=0; i< nbins_ ; i++)
    color[i] = 0.0;

  //Keep binning the histogram based on the color of the points in cloud
  double R=0, G=0, B=0;
  for (size_t i = 0; i < input_->points.size (); i++)
  {
    R = input_->points[i].r;
    G = input_->points[i].g;
    B = input_->points[i].b;
    int ind;
    //Checking if the flag is set to rgb histogram
    if (isrgb_)
    {
      ind = (int)R/bin_interval;
      color[ind]++;
      ind = (int)G/bin_interval;
      color[ind + nbins/3]++;
      ind = (int)B/bin_interval;
      color[ind + 2*nbins/3]++;
    }
    //Checking if the flag is set to yuv histogram
    else if (isyuv_)
    {
      double Y, U, V;
      Y = R * 0.299000 + G * 0.587000 + B * 0.114000;
      U = - R * 0.168736 - G * 0.331264 + B * 0.500000 + 128;
      V = R * 0.500000 - G * 0.418688 - B * 0.081312 + 128;
      ind = (int)Y/bin_interval;
      color[ind]++;
      ind = (int)U/bin_interval;
      color[ind + nbins/3]++;
      ind = (int)V/bin_interval;
      color[ind + 2*nbins/3]++;
    }
    //If the flag is set to hsv histogram
    else
    {
      double r, g, b, h, s, v, d;
      double min_, max_;
      r = (double)R/255.0;
      g = (double)G/255.0;
      b = (double)B/255.0;
      min_ = std::min(r, std::min(g, b));
      max_ = std::max(r, std::max(g, b));
      if (max_ == min_)
      {
        v = max_;
        h = 0;
        s = 0;
      }
      else
      {
        d = (r==min_) ? g-b : ((b==min_) ? r-g : b-r);
        h = (r==min_) ? 3 : ((b==min_) ? 1 : 5);
        h = 60*(h - d/(max_ - min_));
        s = (max_ - min_)/max_;
        v = max_;
      }
      ind = h/bin_interval_h;
      color[ind]++;
      ind = s/bin_interval_s;
      color[ind + nbins/3]++;
      ind = v/bin_interval_v;
      color[ind + 2*nbins/3]++;
    }
  }
  //Initialize the size of the output cloud
  output.points.resize (1);
  output.width = output.height = 1;

  //Copy the histogram to the output point cloud
  for (int i = 0; i < nbins; i++)
  {
    output.points[0].histogram[i] = color[i]/input_->points.size();
  }
}

#define PCL_INSTANTIATE_ColorHEstimation(T,OutT) template class PCL_EXPORTS pcl::ColorHEstimation<T,OutT>;

#endif    // PCL_FEATURES_IMPL_COLORH_H_
