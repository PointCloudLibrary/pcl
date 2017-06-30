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
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_fast_edge_aware.h>

/* This example demonstrates the usage of the fast and edge-aware normal estimation pcl::FastEdgeAwareNormalEstimation. It relies on the
 * 3d edge estimation method pcl::OrganizedEdgeFromPoints which finds 3d depth and surface edges and features the edge-aware normal estimation
 * as a by-product. The normals can be computed very efficiently and their support regions never cross detected edges.
 * The class pcl::FastEdgeAwareNormalEstimation does not give you access to the estimated edges, however, if you are also interested in
 * using the edge labels, please have a look at the example in example_organized_edge_detection.cpp (or at option 3b in this file).
 */

int
main (int argc,
      char** argv)
{
  // 1. open pcd file
  if (argc < 2)
  {
    PCL_ERROR("Error: No pcd file specified via: pcl_example_organized_edge_detection <xyzrgb-pcd-filename>.\n");
    return -1;
  }
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) != 0)  // load the file
  {
    PCL_ERROR("Couldn't read file.\n");
    return -1;
  }
  std::cout << "points: " << cloud->points.size () << std::endl;

  // 2. set up parameters for 3d edge detection
  pcl::EdgeDetectionConfig cfg (pcl::EdgeDetectionConfig::GAUSSIAN, 3, 0.01f, 40., true, 5, 30, 15);
  // ALTERNATIVELY, all these properties can be set individually as follows
  cfg.noise_reduction_mode_ = pcl::EdgeDetectionConfig::GAUSSIAN;
  cfg.noise_reduction_kernel_size_ = 3;
  cfg.depth_step_factor_ = 0.01f;
  cfg.min_detectable_edge_angle_ = 40.;
  cfg.use_adaptive_scan_line_ = true;
  cfg.min_scan_line_width_ = 5;
  cfg.max_scan_line_width_ = 30;
  cfg.scan_line_width_at_2m_ = 15;
  cfg.updateScanLineModel();  // do not forget to run this command after updating the scan line model parameters

  // 3a. compute normals in an efficient and edge-aware fashion
  pcl::StopWatch timer;
  pcl::PointCloud<pcl::Normal>::Ptr normals_edge_aware = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::FastEdgeAwareNormalEstimation<pcl::PointXYZRGB, pcl::Normal> fast_edge_aware_normal_estimation;
  fast_edge_aware_normal_estimation.setEdgeDetectionConfig (cfg);
  fast_edge_aware_normal_estimation.setInputCloud (cloud);
  fast_edge_aware_normal_estimation.compute (*normals_edge_aware);
  std::cout << "Edge detection and normal estimation completed after " << timer.getTime () << "ms." << std::endl;

  // 3b. ALTERNATIVELY, you may compute 3d surface and depth edges and estimate normals at the same time with these commands (see also example_organized_edge_detection.cpp)
  pcl::PointCloud<pcl::Label> edge_labels;
  std::vector<pcl::PointIndices> label_indices;
  pcl::OrganizedEdgeFromPoints<pcl::PointXYZRGB, pcl::Normal, pcl::Label> edge_detection;
  edge_detection.setEdgeDetectionConfig (cfg);
  edge_detection.setInputCloud (cloud);
  edge_detection.setReturnLabelIndices (false);  // if we do not need the label indices vector filled, computations are slightly faster
  edge_detection.setUseFastDepthDiscontinuityMode (true);  // use a specific fast implementation for estimating depth edges, otherwise we can use the standard method of OrganizedEdgeBase
  edge_detection.compute (edge_labels, label_indices, normals_edge_aware);

  // 4. save pcd with computed normals
  pcl::PointCloud<pcl::PointXYZRGBNormal> result;
  pcl::concatenateFields (*cloud, *normals_edge_aware, result);
  std::string pcd_filename = filename + "_normals.pcd";
  pcl::io::savePCDFileBinaryCompressed (pcd_filename, result);
  std::cout << "Saved pcd file with normals to: " << pcd_filename << std::endl;

  // 5. visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals_edge_aware);

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return 0;
}
