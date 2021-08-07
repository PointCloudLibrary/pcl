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

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rift.h>
#include <pcl/features/intensity_gradient.h>

int
main (int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file\n");
    return -1;
  }

  std::cout << "points: " << cloud->size () << std::endl;

  // Estimate the surface normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
  norm_est.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr treept1 (new pcl::search::KdTree<pcl::PointXYZI> (false));
  norm_est.setSearchMethod(treept1);
  norm_est.setRadiusSearch(0.25);
  norm_est.compute(*cloud_n);

  std::cout<<" Surface normals estimated";
  std::cout<<" with size "<< cloud_n->size() <<std::endl;
 
  // Estimate the Intensity Gradient
  pcl::PointCloud<pcl::IntensityGradient>::Ptr cloud_ig (new pcl::PointCloud<pcl::IntensityGradient>);
  pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradient_est;
  gradient_est.setInputCloud(cloud);
  gradient_est.setInputNormals(cloud_n);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr treept2 (new pcl::search::KdTree<pcl::PointXYZI> (false));
  gradient_est.setSearchMethod(treept2);
  gradient_est.setRadiusSearch(0.25);
  gradient_est.compute(*cloud_ig);
  std::cout<<" Intensity Gradient estimated";
  std::cout<<" with size "<< cloud_ig->size() <<std::endl;


  // Estimate the RIFT feature
  pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, pcl::Histogram<32> > rift_est;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr treept3 (new pcl::search::KdTree<pcl::PointXYZI> (false));
  rift_est.setSearchMethod(treept3);
  rift_est.setRadiusSearch(10.0);
  rift_est.setNrDistanceBins (4);
  rift_est.setNrGradientBins (8);
  rift_est.setInputCloud(cloud);
  rift_est.setInputGradient(cloud_ig);
  pcl::PointCloud<pcl::Histogram<32> > rift_output;
  rift_est.compute(rift_output);

  std::cout<<" RIFT feature estimated";
  std::cout<<" with size "<<rift_output.size()<<std::endl;
  
  // Display and retrieve the rift descriptor vector for the first point
  std::cout << rift_output.front() << std::endl;
  return 0;
}
