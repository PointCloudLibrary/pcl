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
 * $Id:$
 *
 */

#include <iostream>

#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>

int
main (int argc, char** av)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Syntax is: %s <source-pcd-file> [-dump]\n\n", av[0]);
    pcl::console::print_info ("If -dump is provided write the extracted clusters to cluster.dat\n\n");
    return (1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nans (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PCDWriter writer;
  if (pcl::io::loadPCDFile(av[1], *cloud_ptr)==-1)
  {
    return -1;
  }

  pcl::console::print_highlight("Loaded cloud %s of size %zu\n",
                                av[1],
                                static_cast<std::size_t>(cloud_ptr->size()));

  // Remove the nans
  cloud_ptr->is_dense = false;
  cloud_no_nans->is_dense = false;
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud (*cloud_ptr, *cloud_no_nans, indices);
  pcl::console::print_highlight("Removed nans from %zu to %zu\n",
                                static_cast<std::size_t>(cloud_ptr->size()),
                                static_cast<std::size_t>(cloud_no_nans->size()));

  // Estimate the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_no_nans);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod (tree_n);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);
  pcl::console::print_highlight("Normals are computed and size is %zu\n",
                                static_cast<std::size_t>(cloud_normals->size()));

  // Region growing
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setSmoothModeFlag (false); // Depends on the cloud being processed
  rg.setInputCloud (cloud_no_nans);
  rg.setInputNormals (cloud_normals);

  std::vector <pcl::PointIndices> clusters;
  pcl::StopWatch watch;
  rg.extract (clusters);
  pcl::console::print_highlight ("Extraction time: %f\n", watch.getTimeSeconds());
  cloud_segmented = rg.getColoredCloud ();

  // Writing the resulting cloud into a pcd file
  pcl::console::print_highlight("Number of segments done is %zu\n",
                                static_cast<std::size_t>(clusters.size()));
  writer.write<pcl::PointXYZRGB> ("segment_result.pcd", *cloud_segmented, false);

  if (pcl::console::find_switch (argc, av, "-dump"))
  {
    pcl::console::print_highlight ("Writing clusters to clusters.dat\n");
    std::ofstream clusters_file;
    clusters_file.open ("clusters.dat");
    for (std::size_t i = 0; i < clusters.size (); ++i)
    {
      clusters_file << i << "#" << clusters[i].indices.size () << ":";
      for (const auto& cluster_idx : clusters[i].indices)
        clusters_file << " " << cluster_idx;
      clusters_file << std::endl;
    }
    clusters_file.close ();
  }

  return (0);
}
