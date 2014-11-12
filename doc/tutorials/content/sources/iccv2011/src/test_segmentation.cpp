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


#include "segmentation.h"

#include <string>
#include <sstream>
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
    pcl::console::print_info ("    -p dist_threshold,max_iters  ..... Subtract the dominant plane\n");
    pcl::console::print_info ("    -c tolerance,min_size,max_size ... Cluster points\n");
    pcl::console::print_info ("    -s output.pcd .................... Save the largest cluster\n");
    return (1);
  }

  // Load the input file
  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], cloud->size ());

  // Subtract the dominant plane
  double dist_threshold, max_iters;
  bool subtract_plane = pcl::console::parse_2x_arguments (argc, argv, "-p", dist_threshold, max_iters) > 0;
  if (subtract_plane)
  {
    size_t n = cloud->size ();
    cloud = findAndSubtractPlane (cloud, dist_threshold, (int)max_iters);
    pcl::console::print_info ("Subtracted %lu points along the detected plane\n", n - cloud->size ());
  }

  // Cluster points
  double tolerance, min_size, max_size;
  std::vector<pcl::PointIndices> cluster_indices;
  bool cluster_points = pcl::console::parse_3x_arguments (argc, argv, "-c", tolerance, min_size, max_size) > 0;
  if (cluster_points)
  {
    clusterObjects (cloud, tolerance, (int)min_size, (int)max_size, cluster_indices);
    pcl::console::print_info ("Found %lu clusters\n", cluster_indices.size ());
  }

  // Save output
  std::string output_filename;
  bool save_cloud = pcl::console::parse_argument (argc, argv, "-s", output_filename) > 0;
  if (save_cloud)
  {
    // If clustering was performed, save only the first (i.e., largest) cluster
    if (cluster_points)
    {
      PointCloudPtr temp_cloud (new PointCloud);
      pcl::copyPointCloud (*cloud, cluster_indices[0], *temp_cloud);
      cloud = temp_cloud;
    }
    pcl::console::print_info ("Saving result as %s...\n", output_filename.c_str ());
    pcl::io::savePCDFile (output_filename, *cloud);
  }
  // Or visualize the result
  else
  {
    pcl::console::print_info ("Starting visualizer... Close window to exit.\n");
    pcl::visualization::PCLVisualizer vis;

    // If clustering was performed, display each cluster with a random color
    if (cluster_points)
    {
      for (size_t i = 0; i < cluster_indices.size (); ++i)
      {
        // Extract the i_th cluster into a new cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_i (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud (*cloud, cluster_indices[i], *cluster_i);

        // Create a random color
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random_color (cluster_i);

        // Create a unique identifier
        std::stringstream cluster_id ("cluster");
        cluster_id << i;

        // Add the i_th cluster to the visualizer with a random color and a unique identifier
        vis.addPointCloud<pcl::PointXYZ> (cluster_i, random_color, cluster_id.str ());
      }
    }
    else
    {
      // If clustering wasn't performed, just display the cloud
      vis.addPointCloud (cloud);
    }
    vis.resetCamera ();
    vis.spin ();
  }

  return (0);
}
