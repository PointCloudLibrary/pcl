/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
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


#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/bilateral_upsampling.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


int default_window_size = 15;
double default_sigma_color = 15;
double default_sigma_depth = 1.5;


void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -window_size X =  (default: ");
  print_value ("%d", default_window_size); print_info (")\n");
  print_info ("                     -sigma_color X =  (default: ");
  print_value ("%f", default_sigma_color); print_info (")\n");
  print_info ("                     -sigma_depth X =  (default: ");
  print_value ("%d", default_sigma_depth); print_info (")\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
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
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         int window_size, double sigma_color, double sigma_depth)
{
  PointCloud<PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<PointXYZRGBA> ());
  fromPCLPointCloud2 (*input, *cloud);

  PointCloud<PointXYZRGBA>::Ptr cloud_upsampled (new PointCloud<PointXYZRGBA> ());

  BilateralUpsampling<PointXYZRGBA, PointXYZRGBA> bu;
  bu.setInputCloud (cloud);
  bu.setWindowSize (window_size);
  bu.setSigmaColor (sigma_color);
  bu.setSigmaDepth (sigma_depth);

  // TODO need to fix this somehow
  bu.setProjectionMatrix (bu.KinectSXGAProjectionMatrix);


  TicToc tt;
  tt.tic ();
  bu.process (*cloud_upsampled);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud_upsampled->width * cloud_upsampled->height); print_info (" points]\n");

  toPCLPointCloud2 (*cloud_upsampled, output);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
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
  print_info ("Bilateral Filter Upsampling of an organized point cloud containing color information. For more information, use: %s -h\n", argv[0]);

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
  int window_size = default_window_size;
  double sigma_color = default_sigma_color;
  double sigma_depth = default_sigma_depth;

  parse_argument (argc, argv, "-window_size", window_size);
  parse_argument (argc, argv, "-sigma_color", sigma_color);
  parse_argument (argc, argv, "-sigma_depth", sigma_depth);

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud))
    return (-1);

  // Do the smoothing
  pcl::PCLPointCloud2 output;
  compute (cloud, output, window_size, sigma_color, sigma_depth);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}
