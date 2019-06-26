/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/io/pcd_io.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s depth.pclzf rgb.pclzf parameters.xml output.pcd\n", argv[0]);
}

bool
loadPCLZF (const std::string &filename_rgb, 
           const std::string &filename_depth,
           const std::string &filename_params,
           pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename_rgb.c_str ());
  tt.tic ();

  pcl::io::LZFRGB24ImageReader rgb;
  pcl::io::LZFBayer8ImageReader bayer;
  pcl::io::LZFYUV422ImageReader yuv;
  pcl::io::LZFDepth16ImageReader depth;

  rgb.readParameters (filename_params);
  bayer.readParameters (filename_params);
  depth.readParameters (filename_params);
  yuv.readParameters (filename_params);

  if (!rgb.read (filename_rgb, cloud))
    if (!yuv.read (filename_rgb, cloud))
      bayer.read (filename_rgb, cloud);

  depth.read (filename_depth, cloud);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

bool
loadPCLZF (const std::string &filename_depth,
           const std::string &filename_params,
           pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename_depth.c_str ());
  tt.tic ();

  pcl::io::LZFDepth16ImageReader depth;
  depth.readParameters (filename_params);
  depth.read (filename_depth, cloud);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

template <typename T> void
saveCloud (const std::string &filename, const pcl::PointCloud<T> &cloud)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::PCDWriter writer;
  writer.writeBinaryCompressed (filename, cloud);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a pair of PCLZF files (depth, rgb) to PCD format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool debug = false;
  pcl::console::parse_argument (argc, argv, "-debug", debug);
  if (debug)
    pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);

  // Parse the command line arguments for .pcd and .ply files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> pclzf_file_indices = parse_file_extension_argument (argc, argv, ".pclzf");
  std::vector<int> xml_file_indices = parse_file_extension_argument (argc, argv, ".xml");
  if (pcd_file_indices.size () != 1 || pclzf_file_indices.empty () || xml_file_indices.size () != 1)
  {
    print_error ("Need at least 1 input PCLZF file, one input XML file, and one output PCD file.\n");
    return (-1);
  }

  std::string filename_depth (argv[pclzf_file_indices[0]]);
  if (pclzf_file_indices.size () > 1)
  {
    std::string filename_rgb (argv[pclzf_file_indices[1]]);

    // Load the data
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    if (!loadPCLZF (filename_rgb, filename_depth, argv[xml_file_indices[0]], cloud)) 
      return (-1);

    // Convert to PCD and save
    saveCloud (argv[pcd_file_indices[0]], cloud);
  }
  else
  {
    // Load the data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (!loadPCLZF (filename_depth, argv[xml_file_indices[0]], cloud)) 
      return (-1);

    // Convert to PCD and save
    saveCloud (argv[pcd_file_indices[0]], cloud);
  }
}

