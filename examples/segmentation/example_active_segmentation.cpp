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
 */

//stl
#include <iostream>

//pcl
#include <pcl/segmentation/active_segmentation.h>
#include <pcl/segmentation/impl/active_segmentation.hpp>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    throw std::runtime_error("Required arguments: filename.pcd");
  }
  std::string fileName = argv[1];
  std::cout << "Reading " << fileName << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileName, *cloud) == -1) // load the file
  {
    PCL_ERROR("Couldn't read file");
    return (-1);
  }
  int fp_indice =atoi(argv[2]);
  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  tree->setInputCloud(cloud);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.02);
  ne.compute(*normals);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr norms(new  pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pcl::concatenateFields(*cloud,*normals,*norms);
  pcl::io::savePCDFile("/tmp/normals.pcd", *norms);

  pcl::PointCloud<pcl::Boundary>::Ptr bps(new pcl::PointCloud<pcl::Boundary>());
  bps->resize(cloud->size());

  //cloud out with boundaries
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
  pcl::copyPointCloud(*cloud,*out_cloud);

  //edge detection in an orgaized Point cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ >());
  pcl::copyPointCloud(*cloud, *cloud_xyz);
  pcl::PointCloud<pcl::Label>::Ptr out_cloud_2(new pcl::PointCloud<pcl::Label>());
  std::vector<pcl::PointIndices> labels;
  pcl::OrganizedEdgeDetection<pcl::PointXYZ,pcl::Label> oed;
  oed.setInputCloud(cloud_xyz);
  oed.compute(*out_cloud_2, labels);

  if(bps->points.size()!= out_cloud->points.size())
  {
    PCL_ERROR("SOMETHING WENT WRONG!. BOUNDARY MAP SHOULD HAVE THE SAME NUMBER OF POINTS AS THE INPUT CLOUD\n");
    exit(0);
  }

  for (unsigned int i = 0; i<out_cloud_2->points.size();++i)
  {
    //if(out_cloud_2->points[i].label> 0 && out_cloud_2->points[i].label != std::numeric_limits<unsigned>::max ())
    if(out_cloud_2->points[i].label != std::numeric_limits<unsigned>::max ())
      bps->points[i].boundary_point = static_cast<unsigned char> (out_cloud_2->points[i].label+1);
    else
      bps->points[i].boundary_point = 0;
  }

  pcl::ActiveSegmentation<pcl::PointXYZRGB, pcl::Normal> as;
  pcl::PointIndices segment;
  as.setInputCloud(cloud);
  as.setInputNormals(normals);
  as.setSearchMethod(tree);
  as.setFixationPoint(fp_indice);
  as.setBoundaryMap(bps);
  as.setSearchRadius(0.01);
  as.segment(segment);

  std::vector<int> fp_indices;
  std::vector<float> fp_dist;

  //this is done so I can see the fixation point more easily.
  tree->nearestKSearch(cloud->points[fp_indice],20,fp_indices,fp_dist);

  for(unsigned int i = 0; i<out_cloud->points.size(); ++i)
  {
    out_cloud->points[i].label = bps->points[i].boundary_point;
    if (std::find(segment.indices.begin(),segment.indices.end(),i) != segment.indices.end() && bps->points[i].boundary_point == 0)
      out_cloud->points[i].label = 8;
    if((i == as.getFixationPointIndex ()) || (std::find(fp_indices.begin(),fp_indices.end(),i)!=fp_indices.end() ))
    {
      out_cloud->points[i].label = 4;
    }
  }

  pcl::io::savePCDFile("/tmp/segmented_boundary.pcd", *out_cloud);
  std::cout<<"SEGMENT SIZE: "<<segment.indices.size()<<std::endl;


}
