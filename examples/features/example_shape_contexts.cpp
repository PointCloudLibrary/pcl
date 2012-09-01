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
 */

#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/impl/3dsc.hpp>
#include <pcl/features/normal_3d.h>

int
main (int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile <pcl::PointXYZ> (filename.c_str (), *cloud) == -1)
  // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }
  std::cout << "Loaded " << cloud->points.size () << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setSearchMethod (kdtree);

  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
  normal_estimation.setRadiusSearch (0.03);
  normal_estimation.compute (*normals);

  // Setup the shape context computation
  pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> shape_context;

  // Provide the point cloud
  shape_context.setInputCloud (cloud);
  // Provide normals
  shape_context.setInputNormals (normals);
  // Use the same KdTree from the normal estimation
  shape_context.setSearchMethod (kdtree);
  pcl::PointCloud<pcl::ShapeContext1980>::Ptr shape_context_features (new pcl::PointCloud<pcl::ShapeContext1980>);

  // The minimal radius is generally set to approx. 1/10 of the search radius, while the pt. density radius is generally set to 1/5
  shape_context.setRadiusSearch (0.2);
  shape_context.setPointDensityRadius (0.04);
  shape_context.setMinimalRadius (0.02);

  // Actually compute the shape contexts
  shape_context.compute (*shape_context_features);
  std::cout << "3DSC output points.size (): " << shape_context_features->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  std::cout << shape_context_features->points[0] << std::endl;
  //float* first_descriptor = shape_context_features->points[0].descriptor; // 1980 elements

  return 0;
}
