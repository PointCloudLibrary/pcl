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

//STL
#include <iostream>

//PCL
#include <pcl/segmentation/active_segmentation.h>
#include <pcl/segmentation/impl/active_segmentation.hpp>
#include <pcl/io/pcd_io.h>

int
main (int argc, char** argv)
{
  typedef pcl::PointXYZRGBA PointType;

  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::Boundary>::Ptr boundary (new pcl::PointCloud<pcl::Boundary> ());
  pcl::PCDWriter writer;

  int min_segment_size = 100;

  if (argc < 3)
    throw std::runtime_error ("Required arguments: filename.pcd index");

  if (pcl::io::loadPCDFile<PointType> (argv[1], *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file %s", argv[1]);
    return (-1);
  }

  unsigned int fp_index = atoi (argv[2]);
  if (fp_index > cloud->size ())
    throw std::runtime_error ("Index of fixation point must be valid");

  //edge detection in an organized point cloud;
  pcl::PointCloud<pcl::Label>::Ptr labels_cloud (new pcl::PointCloud<pcl::Label> ());
  std::vector<pcl::PointIndices> labels_vect;

  if(cloud->height !=1)
  {
    pcl::OrganizedEdgeDetection<PointType, pcl::Label> oed;
    oed.setInputCloud (cloud);
    oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_OCCLUDING);//oed.EDGELABEL_HIGH_CURVATURE
    oed.compute (*labels_cloud, labels_vect);
  }
  else
    throw std::runtime_error ("Input cloud must be organized");

  //estimating normals
  pcl::NormalEstimation<PointType, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<PointType>::Ptr tree_n (new pcl::search::KdTree<PointType> ());
  ne.setSearchMethod (tree_n);
  ne.setRadiusSearch (0.02);
  ne.compute (*cloud_normals);
  std::cout << "Normals are computed and size is " << cloud_normals->points.size () << std::endl;

  boundary->resize( labels_cloud->points.size ());
  for (unsigned int i = 0; i < labels_cloud->points.size (); ++i)
      boundary->points[i].boundary_point = static_cast<unsigned char> (labels_cloud->points[i].label);

  //create Kd tree
  pcl::search::KdTree<PointType>::Ptr tree_as (new pcl::search::KdTree<PointType> ());
  tree_as->setInputCloud(cloud);

  //segment points
  pcl::ActiveSegmentation<PointType, pcl::Normal> as;
  pcl::PointIndices indices_out;
  as.setInputCloud (cloud);
  as.setInputNormals (cloud_normals);
  as.setSearchMethod (tree_as);
  as.setFixationPoint (fp_index);
  as.setBoundaryMap (boundary);
  as.setSearchRadius (0.01);
  //as.segment (indices_out);

  pcl::activeSegmentation<PointType>(*cloud,*boundary,*cloud_normals,tree_as,fp_index,0.01f,indices_out);

  //save segment to pcd file if it contains more points the a certain threshold
  if (indices_out.indices.size() > min_segment_size)
  {
    pcl::PointCloud<PointType>::Ptr cloud_segment (new pcl::PointCloud<PointType> ());
    for (std::vector<int>::const_iterator pit = indices_out.indices.begin (); pit != indices_out.indices.end (); pit++)
      cloud_segment->points.push_back (cloud->points[*pit]);
    cloud_segment->width = static_cast<uint32_t> (cloud_segment->points.size ());
    cloud_segment->height = 1;
    cloud_segment->is_dense = true;

    std::cout << "PointCloud representing the segment has " << cloud_segment->points.size () << " data points." << std::endl;
    writer.write<PointType> ("segment_cloud.pcd", *cloud_segment, false);
  }
  else
    PCL_INFO ("NO Segment found.\n");

}
