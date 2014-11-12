/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include "filters.h"

#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int 
main (int argc, char ** argv)
{
  if (argc < 2) 
  {
    pcl::console::print_info ("Syntax is: %s input.pcd <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -t min_depth,max_depth  ............... Threshold depth\n");
    pcl::console::print_info ("    -d leaf_size  ......................... Downsample\n");
    pcl::console::print_info ("    -r radius,min_neighbors ............... Radius outlier removal\n");
    pcl::console::print_info ("    -s output.pcd ......................... Save output\n");
    return (1);
  }

  // Load the input file
  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], cloud->size ());

  // Threshold depth
  double min_depth, max_depth;
  bool threshold_depth = pcl::console::parse_2x_arguments (argc, argv, "-t", min_depth, max_depth) > 0;
  if (threshold_depth)
  {
    size_t n = cloud->size ();
    cloud = thresholdDepth (cloud, min_depth, max_depth);
    pcl::console::print_info ("Eliminated %lu points outside depth limits\n", n - cloud->size ());
  }

  // Downsample and threshold depth
  double leaf_size;
  bool downsample_cloud = pcl::console::parse_argument (argc, argv, "-d", leaf_size) > 0;
  if (downsample_cloud)
  {
    size_t n = cloud->size ();
    cloud = downsample (cloud, leaf_size);
    pcl::console::print_info ("Downsampled from %lu to %lu points\n", n, cloud->size ());
  }

  // Remove outliers
  double radius, min_neighbors;
  bool remove_outliers = pcl::console::parse_2x_arguments (argc, argv, "-r", radius, min_neighbors) > 0;
  if (remove_outliers)
  {
    size_t n = cloud->size ();
    cloud = removeOutliers (cloud, radius, (int)min_neighbors);
    pcl::console::print_info ("Removed %lu outliers\n", n - cloud->size ());
  }

  // Save output
  std::string output_filename;
  bool save_cloud = pcl::console::parse_argument (argc, argv, "-s", output_filename) > 0;
  if (save_cloud)
  {
    pcl::io::savePCDFile (output_filename, *cloud);
    pcl::console::print_info ("Saved result as %s\n", output_filename.c_str ());
  }
  // Or visualize the result
  else
  {
    pcl::console::print_info ("Starting visualizer... Close window to exit.\n");
    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud (cloud);
    vis.resetCamera ();
    vis.spin ();
  }

  return (0);
}
