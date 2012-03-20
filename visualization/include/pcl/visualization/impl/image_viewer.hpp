/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 *
 * $Id$
 */

#ifndef PCL_VISUALIZATION_IMAGE_VISUALIZER_HPP_
#define	PCL_VISUALIZATION_IMAGE_VISUALIZER_HPP_

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> void
pcl::visualization::ImageViewer::showRGBImage (const pcl::PointCloud<T> &cloud,
                                               const std::string &layer_id,
                                               double opacity)
{
  if (data_size_ < cloud.width * cloud.height)
  {
    data_size_ = cloud.width * cloud.height * 3;
    data_.reset (new unsigned char[data_size_]);
  }
  
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    memcpy (&data_[i * 3], reinterpret_cast<const unsigned char*> (&cloud.points[i].rgba), sizeof (unsigned char) * 3);
    /// Convert from BGR to RGB
    unsigned char aux = data_[i*3];
    data_[i*3] = data_[i*3+2];
    data_[i*3+2] = aux;
    for (int j = 0; j < 3; ++j)
      if (pcl_isnan (data_[i * 3 + j]))
          data_[i * 3 + j] = 0;
  }
  return (showRGBImage (data_.get (), cloud.width, cloud.height, layer_id, opacity));
}


#endif      // PCL_VISUALIZATION_IMAGE_VISUALIZER_HPP_
