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
 * $Id$
 *
 */

#ifndef PCL_POINT_CLOUD_SPRING_IMPL_HPP_
#define PCL_POINT_CLOUD_SPRING_IMPL_HPP_

template <typename PointT> void 
pcl::common::expandColumns (const PointCloud<PointT>& input, PointCloud<PointT>& output, 
                            const PointT& val, const size_t& amount)
{
  if (amount <= 0)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::expandColumns] error: amount must be ]0.."
                         << (input.width/2) << "] !");

  if (!input.isOrganized () || amount > (input.width/2))
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::expandColumns] error: " 
                         << "columns expansion requires organised point cloud");

  uint32_t old_height = input.height;
  uint32_t old_width = input.width;
  uint32_t new_width = old_width + 2*amount;
  if (&input != &output)
    output = input;
  output.reserve (new_width * old_height);
  for (int j = 0; j < output.height; ++j)
  {
    typename PointCloud<PointT>::iterator start = output.begin() + (j * new_width);
    output.insert (start, amount, val);
    start = output.begin() + (j * new_width) + old_width + amount;
    output.insert (start, amount, val);
    output.height = old_height;
  }
  output.width = new_width;
  output.height = old_height;
}

template <typename PointT> void 
pcl::common::expandRows (const PointCloud<PointT>& input, PointCloud<PointT>& output,
                         const PointT& val, const size_t& amount)
{
  if (amount <= 0)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::expandRows] error: amount must be ]0.."
                         << (input.height/2) << "] !");

  uint32_t old_height = input.height;
  uint32_t new_height = old_height + 2*amount;
  uint32_t old_width = input.width;
  if (&input != &output)
    output = input;
  output.reserve (new_height * old_width);
  output.insert (output.begin (), amount * old_width, val);
  output.insert (output.end (), amount * old_width, val);
  output.width = old_width;
  output.height = new_height;
}

template <typename PointT> void 
pcl::common::duplicateColumns (const PointCloud<PointT>& input, PointCloud<PointT>& output,
                               const size_t& amount)
{
  if (amount <= 0)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::duplicateColumns] error: amount must be ]0.."
                         << (input.width/2) << "] !");

  if (!input.isOrganized () || amount > (input.width/2))
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::duplicateColumns] error: " 
                         << "columns expansion requires organised point cloud");

  size_t old_height = input.height;
  size_t old_width = input.width;
  size_t new_width = old_width + 2*amount;
  if (&input != &output)
    output = input;
  output.reserve (new_width * old_height);
  for (size_t j = 0; j < old_height; ++j)
    for(size_t i = 0; i < amount; ++i)
    {
      typename PointCloud<PointT>::iterator start = output.begin () + (j * new_width);
      output.insert (start, *start);
      start = output.begin () + (j * new_width) + old_width + i;
      output.insert (start, *start);
    }

  output.width = new_width;
  output.height = old_height;
}

template <typename PointT> void 
pcl::common::duplicateRows (const PointCloud<PointT>& input, PointCloud<PointT>& output,
                            const size_t& amount)
{
  if (amount <= 0 || amount > (input.height/2))
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::duplicateRows] error: amount must be ]0.." 
                         << (input.height/2) << "] !");

  uint32_t old_height = input.height;
  uint32_t new_height = old_height + 2*amount;
  uint32_t old_width = input.width;
  if (&input != &output)
    output = input;
  output.reserve (new_height * old_width);
  for(size_t i = 0; i < amount; ++i)
  {
    output.insert (output.begin (), output.begin (), output.begin () + old_width);
    output.insert (output.end (), output.end () - old_width, output.end ());
  }

  output.width = old_width;
  output.height = new_height;
}

template <typename PointT> void 
pcl::common::mirrorColumns (const PointCloud<PointT>& input, PointCloud<PointT>& output,
                                  const size_t& amount)
{
  if (amount <= 0)
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::mirrorColumns] error: amount must be ]0.."
                         << (input.width/2) << "] !");

  if (!input.isOrganized () || amount > (input.width/2))
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::mirrorColumns] error: " 
                         << "columns expansion requires organised point cloud");

  size_t old_height = input.height;
  size_t old_width = input.width;
  size_t new_width = old_width + 2*amount;
  if (&input != &output)
    output = input;
  output.reserve (new_width * old_height);
  for (size_t j = 0; j < old_height; ++j)
    for(size_t i = 0; i < amount; ++i)
    {
      typename PointCloud<PointT>::iterator start = output.begin () + (j * new_width);
      output.insert (start, *(start + 2*i));
      start = output.begin () + (j * new_width) + old_width + 2*i;
      output.insert (start+1, *(start - 2*i));
    }
  output.width = new_width;
  output.height = old_height;
}

template <typename PointT> void 
pcl::common::mirrorRows (const PointCloud<PointT>& input, PointCloud<PointT>& output,
                         const size_t& amount)
{
  if (amount <= 0 || amount > (input.height/2))
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::mirrorRows] error: amount must be ]0.." 
                         << (input.height/2) << "] !");

  uint32_t old_height = input.height;
  uint32_t new_height = old_height + 2*amount;
  uint32_t old_width = input.width;
  if (&input != &output)
    output = input;
  output.reserve (new_height * old_width);
  for(size_t i = 0; i < amount; i++)
  {
    typename PointCloud<PointT>::iterator up;
    if (output.height % 2 ==  0)
      up = output.begin () + (2*i) * old_width;
    else
      up = output.begin () + (2*i+1) * old_width;
    output.insert (output.begin (), up, up + old_width);
    typename PointCloud<PointT>::iterator bottom = output.end () - (2*i+1) * old_width;
    output.insert (output.end (), bottom, bottom + old_width);
  }
  output.width = old_width;
  output.height = new_height;
}

template <typename PointT> void 
pcl::common::deleteRows (const PointCloud<PointT>& input, PointCloud<PointT>& output,
                         const size_t& amount)
{
  if (amount <= 0 || amount > (input.height/2))
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::deleteRows] error: amount must be ]0.." 
                         << (input.height/2) << "] !");

  uint32_t old_height = input.height;
  uint32_t old_width = input.width;
  output.erase (output.begin (), output.begin () + amount * old_width);
  output.erase (output.end () - amount * old_width, output.end ());
  output.height = old_height - 2*amount;
  output.width = old_width;
}

template <typename PointT> void 
pcl::common::deleteCols (const PointCloud<PointT>& input, PointCloud<PointT>& output,
                         const size_t& amount)
{
  if (amount <= 0 || amount > (input.width/2))
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::deleteCols] error: amount must be in ]0.."
                         << (input.width/2) << "] !");

  if (!input.isOrganized ())
    PCL_THROW_EXCEPTION (InitFailedException,
                         "[pcl::common::deleteCols] error: " 
                         << "columns delete requires organised point cloud");

  uint32_t old_height = input.height;
  uint32_t old_width = input.width;
  uint32_t new_width = old_width - 2 * amount;
  for(size_t j = 0; j < old_height; j++)
  {
    typename PointCloud<PointT>::iterator start = output.begin () + j * new_width;
    output.erase (start, start + amount);
    start = output.begin () + (j+1) * new_width;
    output.erase (start, start + amount);    
  }
  output.height = old_height;
  output.width = new_width;
}

#endif
