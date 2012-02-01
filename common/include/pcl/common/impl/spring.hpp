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
 *
 * $Id$
 *
 */

#ifndef PCL_POINT_CLOUD_SPRING_IMPL_HPP_
#define PCL_POINT_CLOUD_SPRING_IMPL_HPP_

template <typename PointT> inline bool
pcl::PointCloudSpring<PointT>::initCompute (PointCloud& output)
{
  if ((expand_policy_ != MIRROR) && (expand_policy_ != DUPLICATE))
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudSpring::initCompute] init failed: " 
                         << "expansion policy is either MIRROR or DUPLICATE");
    return false;
  }

  if (c_amount_ <= 0 && r_amount_ <=0)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudSpring::initCompute] init failed: " 
                         << "expansion amount must be strict positive!");
    return false;
  }

  if (!input_->isOrganized () && c_amount_ > 0)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudSpring::initCompute] init failed: " 
                         << "columns expansion requires organised point cloud");
    return false;
  }
  
  if (&(*input_) != &output)
    output = *input_;

  return true;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandColumns (const PointT& val, PointCloud& output)
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  uint32_t new_width = old_width + 2*c_amount_;
  output.reserve (new_width * old_height);
  for (int j = 0; j < output.height; ++j)
  {
    iterator start = output.begin() + (j * new_width);
    output.insert (start, c_amount_, val);
    start = output.begin() + (j * new_width) + old_width + c_amount_;
    output.insert (start, c_amount_, val);
    output.height = old_height;
  }
  output.width = new_width;
  output.height = old_height;
}
      
template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandRows (const PointT& val, PointCloud& output)
{
  uint32_t old_height = input_->height;
  uint32_t new_height = old_height + 2*r_amount_;
  uint32_t old_width = input_->width;
  output.reserve (new_height * old_width);
  output.insert (output.begin (), r_amount_ * old_width, val);
  output.insert (output.end (), r_amount_ * old_width, val);
  output.width = old_width;
  output.height = new_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandColumnsDuplicate (PointCloud& output)
{
  int old_height = input_->height;
  int old_width = input_->width;
  int new_width = old_width + 2*c_amount_;
  output.reserve (new_width * old_height);
  for (int j = 0; j < old_height; ++j)
    for(int i = 0; i < c_amount_; ++i)
    {
      iterator start = output.begin () + (j * new_width);
      output.insert (start, *start);
      start = output.begin () + (j * new_width) + old_width + i;
      output.insert (start, *start);
    }

  output.width = new_width;
  output.height = old_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandRowsDuplicate (PointCloud& output)
{
  uint32_t old_height = input_->height;
  uint32_t new_height = old_height + 2*r_amount_;
  uint32_t old_width = input_->width;
  output.reserve (new_height * old_width);
  for(int i = 0; i < r_amount_; ++i)
  {
    output.insert (output.begin (), output.begin (), output.begin () + old_width);
    output.insert (output.end (), output.end () - old_width, output.end ());
  }

  output.width = old_width;
  output.height = new_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandColumnsMirror (PointCloud& output)
{
  int old_height = input_->height;
  int old_width = input_->width;
  int new_width = old_width + 2*c_amount_;
  output.reserve (new_width * old_height);
  for (int j = 0; j < old_height; ++j)
    for(int i = 0; i < c_amount_; ++i)
    {
      iterator start = output.begin () + (j * new_width);
      output.insert (start, *(start + 2*i));
      start = output.begin () + (j * new_width) + old_width + 2*i;
      output.insert (start+1, *(start - 2*i));
    }
  output.width = new_width;
  output.height = old_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandRowsMirror (PointCloud& output)
{
  uint32_t old_height = input_->height;
  uint32_t new_height = old_height + 2*r_amount_;
  uint32_t old_width = input_->width;
  output.reserve (new_height * old_width);
  for(int i = 0; i < r_amount_; i++)
  {
    iterator up;
    if (output.height % 2 ==  0)
      up = output.begin () + (2*i) * old_width;
    else
      up = output.begin () + (2*i+1) * old_width;
    output.insert (output.begin (), up, up + old_width);
    iterator bottom = output.end () - (2*i+1) * old_width;
    output.insert (output.end (), bottom, bottom + old_width);
  }
  output.width = old_width;
  output.height = new_height;
}

template <typename PointT> inline void 
pcl::PointCloudSpring<PointT>::deleteRows (PointCloud& output)
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  output.erase (output.begin (), output.begin () + r_amount_ * old_width);
  output.erase (output.end () - r_amount_ * old_width, output.end ());
  output.height = old_height - 2*r_amount_;
  output.width = old_width;
}

template <typename PointT> inline void 
pcl::PointCloudSpring<PointT>::deleteCols (PointCloud& output)
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  uint32_t new_width = old_width - 2 * c_amount_;
  for(uint32_t j = 0; j < old_height; j++)
  {
    iterator start = output.begin () + j * new_width;
    output.erase (start, start + c_amount_);
    start = output.begin () + (j+1) * new_width;
    output.erase (start, start + c_amount_);    
  }
  output.height = old_height;
  output.width = new_width;
}

#endif
