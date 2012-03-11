/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: example_removeNaNFromPointCloud.cpp 4117 2012-01-31 17:56:02Z aichim $
 *
 */

// STL
#include <iostream>
#include <limits>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

int
main (int, char**)
{
  typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  CloudType::Ptr cloud (new CloudType);
  cloud->is_dense = false;
  CloudType::Ptr output_cloud (new CloudType);

  CloudType::PointType p_nan;
  p_nan.x = std::numeric_limits<float>::quiet_NaN();
  p_nan.y = std::numeric_limits<float>::quiet_NaN();
  p_nan.z = std::numeric_limits<float>::quiet_NaN();
  cloud->push_back(p_nan);

  CloudType::PointType p_valid;
  p_valid.x = 1.0f;
  cloud->push_back(p_valid);

  std::cout << "size: " << cloud->points.size () << std::endl;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *output_cloud, indices);
  std::cout << "size: " << output_cloud->points.size () << std::endl;

  return 0;
}
