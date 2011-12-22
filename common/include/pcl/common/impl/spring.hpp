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

  if ((direction_ != HORIZONTAL) && (direction_ != VERTICAL) && (direction_ != BOTH))
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudSpring::initCompute] init failed: "
                         << "border must be a HORIZONTAL, VERTICAL or BOTH");
    return false;
  }

  if (amount_ <= 0)
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudSpring::initCompute] init failed: " 
                         << "expansion amount must be strict positive!");
    return false;
  }
        
  if (!input_->isOrganized () && (expand_policy_ == VERTICAL || expand_policy_ == BOTH))
  {
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::PointCloudSpring::initCompute] init failed: " 
                         << "vertical expansion requires organised point cloud");
    return false;
  }
  return true;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandHorizontal(const PointT& val)
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
pcl::PointCloudSpring<PointT>::expandVertical(const PointT& val)
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
pcl::PointCloudSpring<PointT>::expandHorizontalDuplicate()
{
  int old_height = input_->height;
  int old_width = input_->width;
  int new_width = old_width + 2*amount_;
  for (int j = 0; j < old_height; ++j)
    for(int i = 0; i < amount_; ++i)
    {
      iterator start = input_->begin () + (j * new_width);
      // For some reason the dereferenced iterator from eigen aligned 
      // vector don't return the actual point!!?
      // input_->insert (start, *start);
      input_->insert (start, input_->at (size_t (std::distance (input_->begin (), start))));
      start = input_->begin () + (j * new_width) + old_width + i;
      input_->insert (start, *start);
    }

  input_->width = old_width + 2*amount_;
  input_->height = old_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandVerticalDuplicate()
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
pcl::PointCloudSpring<PointT>::expandHorizontalMirror()
{
  int old_height = input_->height;
  int old_width = input_->width;
  int new_width = old_width + 2*amount_;
  for (int j = 0; j < old_height; ++j)
    for(int i = 0; i < amount_; ++i)
    {
      iterator start = input_->begin () + (j * new_width);
      input_->insert (start, *(start + 2*i));
      start = input_->begin () + (j * new_width) + old_width + 2*i;
      input_->insert (start+1, *(start - 2*i));
    }
  input_->width = old_width + 2*amount_;
  input_->height = old_height;
}

template <typename PointT> void 
pcl::PointCloudSpring<PointT>::expandVerticalMirror()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;

  iterator up = input_->begin (), low = input_->end ();
  for(int i = 0; i < amount_; ++i)
  {
    input_->insert (input_->begin (), up, up + old_width);
    up+= old_width;
    input_->insert (input_->end (), low - old_width, low);
    low-= old_width;
  }
  input_->width = old_width;
  input_->height = old_height + 2*amount_;
}

template <typename PointT> inline void 
pcl::PointCloudSpring<PointT>::deleteRows ()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  input_->erase (input_->begin (), input_->begin () + amount_ * old_width);
  input_->erase (input_->end () - amount_ * old_width, input_->end ());
  input_->height = old_height - 2*amount_;
  input_->width = old_width;
}

template <typename PointT> inline void 
pcl::PointCloudSpring<PointT>::deleteCols ()
{
  uint32_t old_height = input_->height;
  uint32_t old_width = input_->width;
  uint32_t new_width = old_width - 2 * amount_;
  for(uint32_t j = 0; j < old_height; j++)
  {
    iterator start = input_->begin () + j * new_width;
    input_->erase (start, start + amount_);
    start = input_->begin () + (j+1) * new_width;
    input_->erase (start, start + amount_);    
  }
  input_->height = old_height;
  input_->width = new_width;
}

#endif
