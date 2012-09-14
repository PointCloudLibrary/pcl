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
 * $Id$
 *
 *
 */

// STL
#include <iostream>

// PCL 
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree.h>

int
main (int, char** av)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nans (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PCDWriter writer;
  if (pcl::io::loadPCDFile(av[1], *cloud_ptr)==-1)
  {
    std::cout << "Couldn't find the file " << av[1] << std::endl;
    return -1;
  }
  
  std::cout << "Loaded cloud " << av[1] << " of size " << cloud_ptr->points.size() << std::endl;	

  // Remove the nans
  cloud_ptr->is_dense = false;
  cloud_no_nans->is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*cloud_ptr, *cloud_no_nans, indices);
  std::cout << "Removed nans from " << cloud_ptr->points.size () << " to " << cloud_no_nans->points.size () << std::endl;

  // Estimate the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_no_nans);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod (tree_n);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);
  std::cout << "Normals are computed and size is " << cloud_normals->points.size () << std::endl;

  // Region growing
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setSmoothModeFlag (false); // Depends on the cloud being processed
  rg.setInputCloud (cloud_no_nans);
  rg.setInputNormals (cloud_normals);

  std::vector <pcl::PointIndices> clusters;
  rg.extract (clusters);

  cloud_segmented = rg.getColoredCloud ();

  // Writing the resulting cloud into a pcd file
  std::cout << "No of segments done is " << clusters.size () << std::endl;
  writer.write<pcl::PointXYZRGB> ("segment_result.pcd", *cloud_segmented, false);

  return (0);
}
