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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>


int 
main (int, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PCDWriter writer;
	
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_ptr) == -1)
  {
    std::cout<<"Couldn't read the file "<<argv[1]<<std::endl;
    return (-1);
  }
  std::cout << "Loaded pcd file " << argv[1] << " with " << cloud_ptr->points.size () << std::endl;

  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_ptr);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod (tree_n);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);
  std::cout << "Estimated the normals" << std::endl;

  // Creating the kdtree object for the search method of the extraction
  boost::shared_ptr<pcl::KdTree<pcl::PointXYZ> > tree_ec  (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  tree_ec->setInputCloud (cloud_ptr);
  
  // Extracting Euclidean clusters using cloud and its normals
  std::vector<int> indices;
  std::vector<pcl::PointIndices> cluster_indices;
  const float tolerance = 0.5f; // 50cm tolerance in (x, y, z) coordinate system
  const double eps_angle = 5 * (M_PI / 180.0); // 5degree tolerance in normals
  const unsigned int min_cluster_size = 50;
 
  pcl::extractEuclideanClusters (*cloud_ptr, *cloud_normals, tolerance, tree_ec, cluster_indices, eps_angle, min_cluster_size);

  std::cout << "No of clusters formed are " << cluster_indices.size () << std::endl;

  // Saving the clusters in seperate pcd files
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_ptr->points[*pit]); 
    cloud_cluster->width = static_cast<uint32_t> (cloud_cluster->points.size ());
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster using xyzn: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "./cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
    j++;
  }

  return (0);
}
