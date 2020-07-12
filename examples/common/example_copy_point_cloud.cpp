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
 * $Id: example_CopyPointCloud.cpp 4117 2012-01-31 17:56:02Z aichim $
 *
 */

/* This example demonstrates how to copy a point cloud. The destination
 * cloud is not required to have the same PointType as the source cloud,
 * but of course the destination PointType must have a superset of the
 * members of the source PointType.
 */

#include <iostream>

#include <pcl/common/io.h>

static void
sameType ();

static void
differenceType ();

// We show this function as an example of what you cannot do.
// static void
// badConversion ();

int
main ()
{
  sameType();
  differenceType();
  // badConversion();
  return 0;
}

void
sameType ()
{
  using CloudType = pcl::PointCloud<pcl::PointXYZ>;
  CloudType::Ptr cloud (new CloudType);

  CloudType::PointType p;
  p.x = 1; p.y = 2; p.z = 3;
  cloud->push_back(p);
  std::cout << p.x << " " << p.y << " " << p.z << std::endl;

  CloudType::Ptr cloud2(new CloudType);
  copyPointCloud(*cloud, *cloud2);

  CloudType::PointType p_retrieved = (*cloud2)[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z << std::endl;
}

void
differenceType ()
{
  using CloudType = pcl::PointCloud<pcl::PointXYZ>;
  CloudType::Ptr cloud (new CloudType);

  CloudType::PointType p;
  p.x = 1; p.y = 2; p.z = 3;
  cloud->push_back(p);
  std::cout << p.x << " " << p.y << " " << p.z << std::endl;

  using CloudType2 = pcl::PointCloud<pcl::PointNormal>;
  CloudType2::Ptr cloud2(new CloudType2);
  copyPointCloud(*cloud, *cloud2);

  CloudType2::PointType p_retrieved = (*cloud2)[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z << std::endl;
}

/*
// Of course you can't do this, because pcl::Normal does not have x,y,z members:
error: ‘pcl::PointCloud<pcl::Normal>::PointType’ has no member named ‘x’

void
badConversion ()
{
  using CloudType = pcl::PointCloud<pcl::PointXYZ>;
  CloudType::Ptr cloud (new CloudType);
  
  CloudType::PointType p;
  p.x = 1; p.y = 2; p.z = 3;
  cloud->push_back(p);
  std::cout << p.x << " " << p.y << " " << p.z << std::endl;

  using CloudType2 = pcl::PointCloud<pcl::Normal>;
  CloudType2::Ptr cloud2(new CloudType2);
  copyPointCloud(*cloud, *cloud2);
  
  CloudType2::PointType p_retrieved = (*cloud2)[0];
  std::cout << p_retrieved.x << " " << p_retrieved.y << " " << p_retrieved.z << std::endl;
}

*/
