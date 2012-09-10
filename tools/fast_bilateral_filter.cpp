/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.

 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include <pcl/filters/fast_bilateral.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_sigma_s = 15.0f;
float default_sigma_r = 0.05f;
bool default_early_division = false;

void
printHelp (int, char **argv)
{
  print_info ("Note: only works with organized point clouds\n");
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -sigma_s X = size of the filter window (default: ");
  print_value ("%f", default_sigma_s); print_info (")\n");
  print_info ("                     -sigma_r X = standard deviation of the filter (default: ");
  print_value ("%f", default_sigma_r); print_info (")\n");
  print_info ("                     -early_division X = early division (default: ");
  print_value ("%d", default_early_division); print_info (")\n");
}

bool
loadCloud (const std::string &filename, sensor_msgs::PointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}


void
compute (const sensor_msgs::PointCloud2::ConstPtr &input, sensor_msgs::PointCloud2 &output,
         float sigma_s, float sigma_r, bool early_division)
{
  PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA> ());
  fromROSMsg (*input, *cloud);

  FastBilateralFilter<PointXYZRGBA> filter;
  filter.setSigmaS (sigma_s);
  filter.setSigmaR (sigma_r);
  filter.setEarlyDivision (early_division);
  filter.setInputCloud (cloud);

  PointCloud<PointXYZRGBA>::Ptr out_cloud (new PointCloud<PointXYZRGBA> ());


  TicToc tt;
  tt.tic ();
  filter.filter (*out_cloud);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", out_cloud->width * out_cloud->height); print_info (" points]\n");

  toROSMsg (*out_cloud, output);
}

void
saveCloud (const std::string &filename, const sensor_msgs::PointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFile (filename, output,  Eigen::Vector4f::Zero (),
      Eigen::Quaternionf::Identity (), true);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Moving Least Squares smoothing of a point cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing
  float sigma_s = default_sigma_s;
  float sigma_r = default_sigma_r;
  bool early_division = default_early_division;

  parse_argument (argc, argv, "-sigma_s", sigma_s);
  parse_argument (argc, argv, "-sigma_r", sigma_r);
  parse_argument (argc, argv, "-early_division", early_division);

  // Load the first file
  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud))
    return (-1);

  // Apply the filter
  sensor_msgs::PointCloud2 output;
  compute (cloud, output, sigma_s, sigma_r, early_division);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}
