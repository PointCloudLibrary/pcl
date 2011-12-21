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

#ifndef PCL_POINT_CLOUD_EXPANDER_IMPL_HPP_
#define PCL_POINT_CLOUD_EXPANDER_IMPL_HPP_

template <typename PointT> inline bool
pcl::PointCloudExpander<PointT>::initCompute ()
{
  if ((expand_policy_ != MIRROR) && (expand_policy_ != DUPLICATE))
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudExpander::initCompute] init failed: " 
                         << "expansion policy is either MIRROR or DUPLICATE");
    return false;
  }

  if ((direction_ != HORIZONTAL) && (direction_ != VERTICAL) && (direction_ != BOTH))
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudExpander::initCompute] init failed: "
                         << "border must be a HORIZONTAL, VERTICAL or BOTH");
    return false;
  }

  if (amount_ <= 0)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudExpander::initCompute] init failed: " 
                         << "expansion amount must be strict positive!");
    return false;
  }
        
  if (expand_policy_ & VERTICAL && !input_->isOrganized ())
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudExpander::initCompute] init failed: " 
                         << "vertical expansion requires organised point cloud");
    return false;
  }
  return true;
}


template <typename PointT> void 
pcl::PointCloudExpander<PointT>::expandHorizontal(const PointT& val)
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  uint32_t new_width = old_width + 2*amount_;
  for (int j = 0; j < input_->height; ++j)
  {
    iterator start = input_->begin() + (j * new_width);
    input_->insert (start, amount_, val);
    start = input_->begin() + (j * new_width) + old_width + amount_;
    input_->insert (start, amount_, val);
    input_->height = old_height;
  }
  input_->width = old_width + 2*amount_;
  input_->height = old_height;
}
      
template <typename PointT> void 
pcl::PointCloudExpander<PointT>::expandVertical(const PointT& val)
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  input_->insert (input_->begin (), amount_ * old_width, val);
  iterator last = input_->end () -1;
  input_->insert (last, amount_ * old_width, val);
  input_->width = old_width;
  input_->height = old_height + 2*amount_;
}

template <typename PointT> void 
pcl::PointCloudExpander<PointT>::expandHorizontalDuplicate()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  uint32_t new_width = old_width + 2*amount_;
  for (int j = 0; j < old_height; ++j)
  {
    for(int i = 0; i < amount_; ++i)
    {
      iterator start = input_->begin () + (j * new_width);
      input_->insert (start, *start);
      start = input_->begin () + (j * new_width) + old_width + i+1;
      input_->insert (start, *start);
    }
  }
  input_->width = old_width + 2*amount_;
  input_->height = old_height;
}

template <typename PointT> void 
pcl::PointCloudExpander<PointT>::expandVerticalDuplicate()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  for(int i = 0; i < amount_; ++i)
  {
    input_->insert (input_->begin (), input_->begin (), input_->begin () + old_width);
    input_->insert (input_->end (), input_->end () - old_width, input_->end ());
  }
  input_->width = old_width;
  input_->height = old_height + 2*amount_;
}

template <typename PointT> void 
pcl::PointCloudExpander<PointT>::expandHorizontalMirror()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  uint32_t new_width = old_width + 2*amount_;
  for (int j = 0; j < old_height; ++j)
  {
    for(int i = 0; i < amount_; ++i)
    {
      iterator start = input_->begin () + (j * new_width);
      input_->insert (start, *start);
      start = input_->begin () + (j * new_width) + old_width + i+1;
      input_->insert (start, *start);
    }
  }
  input_->width = old_width + 2*amount_;
  input_->height = old_height;
}

template <typename PointT> void 
pcl::PointCloudExpander<PointT>::expandVerticalMirror()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;

  iterator up = input_->begin (), low = input_->end ();
  for(int i = 0; i < amount_; ++i)
  {
    // std::cout << "first " << int (std::distance (input_->begin (),first)) / old_width << std::endl;
    // std::cout << "last " << int (std::distance (input_->begin (),first + old_width)) / old_width << std::endl;
    input_->insert (input_->begin (), up, up + old_width);
    up+= old_width;
    input_->insert (input_->end (), low - old_width, low);
    low-= old_width;
    // first = input_->end () - (2 * i * old_width);
    // std::cout << "first2 " << int (std::distance (input_->begin (),first)) / old_width << std::endl;
    // input_->insert (input_->end (), first, first + old_width);
  }
  input_->width = old_width;
  input_->height = old_height + 2*amount_;
}

#endif
