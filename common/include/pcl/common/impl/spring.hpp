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
pcl::PointCloudSpring<PointT>::initCompute ()
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
  return true;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandColumns (const PointT& val)
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  uint32_t new_width = old_width + 2*c_amount_;
  input_->reserve (new_width * old_height);
  for (int j = 0; j < input_->height; ++j)
  {
    iterator start = input_->begin() + (j * new_width);
    input_->insert (start, c_amount_, val);
    start = input_->begin() + (j * new_width) + old_width + c_amount_;
    input_->insert (start, c_amount_, val);
    input_->height = old_height;
  }
  input_->width = new_width;
  input_->height = old_height;
}
      
template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandRows (const PointT& val)
{
  uint32_t old_height = input_->height;
  uint32_t new_height = old_height + 2*r_amount_;
  uint32_t old_width = input_->width;
  input_->reserve (new_height * old_width);
  input_->insert (input_->begin (), r_amount_ * old_width, val);
  input_->insert (input_->end (), r_amount_ * old_width, val);
  input_->width = old_width;
  input_->height = new_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandColumnsDuplicate ()
{
  int old_height = input_->height;
  int old_width = input_->width;
  int new_width = old_width + 2*c_amount_;
  input_->reserve (new_width * old_height);
  for (int j = 0; j < old_height; ++j)
    for(int i = 0; i < c_amount_; ++i)
    {
      iterator start = input_->begin () + (j * new_width);
      input_->insert (start, *start);
      start = input_->begin () + (j * new_width) + old_width + i;
      input_->insert (start, *start);
    }

  input_->width = new_width;
  input_->height = old_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandRowsDuplicate ()
{
  uint32_t old_height = input_->height;
  uint32_t new_height = old_height + 2*r_amount_;
  uint32_t old_width = input_->width;
  input_->reserve (new_height * old_width);
  for(int i = 0; i < r_amount_; ++i)
  {
    input_->insert (input_->begin (), input_->begin (), input_->begin () + old_width);
    input_->insert (input_->end (), input_->end () - old_width, input_->end ());
  }

  input_->width = old_width;
  input_->height = new_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandColumnsMirror ()
{
  int old_height = input_->height;
  int old_width = input_->width;
  int new_width = old_width + 2*c_amount_;
  input_->reserve (new_width * old_height);
  for (int j = 0; j < old_height; ++j)
    for(int i = 0; i < c_amount_; ++i)
    {
      iterator start = input_->begin () + (j * new_width);
      input_->insert (start, *(start + 2*i));
      start = input_->begin () + (j * new_width) + old_width + 2*i;
      input_->insert (start+1, *(start - 2*i));
    }
  input_->width = new_width;
  input_->height = old_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandRowsMirror ()
{
  uint32_t old_height = input_->height;
  uint32_t new_height = old_height + 2*r_amount_;
  uint32_t old_width = input_->width;
  input_->reserve (new_height * old_width);
  for(int i = 0; i < r_amount_; i++)
  {
    iterator up;
    if (input_->height % 2 ==  0)
      up = input_->begin () + (2*i) * old_width;
    else
      up = input_->begin () + (2*i+1) * old_width;
    input_->insert (input_->begin (), up, up + old_width);
    iterator bottom = input_->end () - (2*i+1) * old_width;
    input_->insert (input_->end (), bottom, bottom + old_width);
  }
  input_->width = old_width;
  input_->height = new_height;
}

template <typename PointT> inline void 
pcl::PointCloudSpring<PointT>::deleteRows ()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  input_->erase (input_->begin (), input_->begin () + r_amount_ * old_width);
  input_->erase (input_->end () - r_amount_ * old_width, input_->end ());
  input_->height = old_height - 2*r_amount_;
  input_->width = old_width;
}

template <typename PointT> inline void 
pcl::PointCloudSpring<PointT>::deleteCols ()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  uint32_t new_width = old_width - 2 * c_amount_;
  for(uint32_t j = 0; j < old_height; j++)
  {
    iterator start = input_->begin () + j * new_width;
    input_->erase (start, start + c_amount_);
    start = input_->begin () + (j+1) * new_width;
    input_->erase (start, start + c_amount_);    
  }
  input_->height = old_height;
  input_->width = new_width;
}

#endif
