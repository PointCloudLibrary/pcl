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

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/auto_io.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <string>
#include <pcl/io/vtk_io.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

double default_radius = 0.01;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s <input_point_cloud> <output_point_cloud> <options>\n", argv[0]);
  print_info ("This tool rely on the file extensions to guess the good reader/writer.\n");
  print_info ("The supported extension for the point cloud are .pcd .ply and .vtk\n");
  print_info ("  where options are:\n");
  print_info ("                     -radius X = use a leaf size of X,X,X to uniformly select 1 point per leaf (default: "); 
  print_value ("%f", default_radius); print_info (")\n");
}

bool
loadCloud (const string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (pcl::io::load (filename, cloud)) {
    print_error ("Cannot found input file name (%s).\n", filename.c_str ());
    return (false);
  }
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
  print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         double radius)
{
  // Convert data to PointCloud<T>
  PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>);
  fromPCLPointCloud2 (*input, *xyz);

  // Estimate
  TicToc tt;
  tt.tic ();
  
  print_highlight (stderr, "Computing ");

  UniformSampling<PointXYZ> us;
  us.setInputCloud (xyz);
  us.setRadiusSearch (radius);
  PointCloud<PointXYZ> output_;
  us.filter (output_);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
  print_value ("%d", output_.size()); print_info (" points]\n");

  // Convert data back
  toPCLPointCloud2 (output_, output);
}

void
saveCloud (const string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w_pcd;
  PLYWriter w_ply;
  std::string output_ext = boost::filesystem::extension (filename);
  std::transform (output_ext.begin (), output_ext.end (), output_ext.begin (), ::tolower);

  if (output_ext.compare (".pcd") == 0)
  {
    w_pcd.writeBinaryCompressed (filename, output);
  }
  else if (output_ext.compare (".ply") == 0)
  {
    w_ply.writeBinary (filename, output);
  }
  else if (output_ext.compare (".vtk") == 0)
  {
    w_ply.writeBinary (filename, output);
  }
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Uniform subsampling using UniformSampling. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  vector<int> p_file_indices;
  vector<std::string> extension;
  extension.emplace_back(".pcd");
  extension.emplace_back(".ply");
  extension.emplace_back(".vtk");
  p_file_indices = parse_file_extension_argument (argc, argv, extension);

  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input file and one output file to continue.\n");
    return (-1);
  }

  std::string input_filename = argv[p_file_indices[0]];
  std::string output_filename = argv[p_file_indices[1]];

  // Command line parsing
  double radius = default_radius;
  parse_argument (argc, argv, "-radius", radius);
  print_info ("Extracting uniform points with a leaf size of: "); 
  print_value ("%f\n", radius); 

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (input_filename, *cloud))
    return (-1);

  // Perform the keypoint estimation
  pcl::PCLPointCloud2 output;
  compute (cloud, output, radius);

  // Save into the second file
  saveCloud (output_filename, output);
}
