/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float       default_leaf_size = 0.01f;
std::string default_field ("z");
double      default_filter_min = -std::numeric_limits<double>::max ();
double      default_filter_max = std::numeric_limits<double>::max ();

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -leaf x,y,z   = the VoxelGrid leaf size (default: "); 
  print_value ("%f, %f, %f", default_leaf_size, default_leaf_size, default_leaf_size); print_info (")\n");
  print_info ("                     -field X      = filter data along this field name (default: "); 
  print_value ("%s", default_field.c_str ()); print_info (")\n");
  print_info ("                     -fmin  X      = filter all data with values along the specified field smaller than this value (default: "); 
  print_value ("-inf"); print_info (")\n");
  print_info ("                     -fmax  X      = filter all data with values along the specified field larger than this value (default: "); 
  print_value ("inf"); print_info (")\n");
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
         float leaf_x, float leaf_y, float leaf_z, const std::string &field, double fmin, double fmax)
{
  TicToc tt;
  tt.tic ();
  
  print_highlight ("Computing ");

  VoxelGrid<pcl::PCLPointCloud2> grid;
  grid.setInputCloud (input);
  grid.setFilterFieldName (field);
  grid.setFilterLimits (fmin, fmax);
  grid.setLeafSize (leaf_x, leaf_y, leaf_z);
  grid.filter (output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Downsample a cloud using pcl::VoxelGrid. For more information, use: %s -h\n", argv[0]);

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
  float leaf_x = default_leaf_size,
        leaf_y = default_leaf_size,
        leaf_z = default_leaf_size;

  std::vector<double> values;
  parse_x_arguments (argc, argv, "-leaf", values);
  if (values.size () == 1)
  {
    leaf_x = static_cast<float> (values[0]);
    leaf_y = static_cast<float> (values[0]);
    leaf_z = static_cast<float> (values[0]);
  }
  else if (values.size () == 3)
  {
    leaf_x = static_cast<float> (values[0]);
    leaf_y = static_cast<float> (values[1]);
    leaf_z = static_cast<float> (values[2]);
  }
  else
  {
    print_error ("Leaf size must be specified with either 1 or 3 numbers (%zu given).\n", values.size ());
  }
  print_info ("Using a leaf size of: "); print_value ("%f, %f, %f\n", leaf_x, leaf_y, leaf_z);

  std::string field (default_field);
  parse_argument (argc, argv, "-field", field);
  double fmin = default_filter_min,
         fmax = default_filter_max;
  parse_argument (argc, argv, "-fmin", fmin);
  parse_argument (argc, argv, "-fmax", fmax);
  print_info ("Filtering data on field: "); print_value ("%s", field.c_str ()); print_info (" between: "); 
  if (fmin == -std::numeric_limits<double>::max ())
    print_value ("-inf ->");
  else
    print_value ("%f ->", fmin);
  if (fmax == std::numeric_limits<double>::max ())
    print_value ("inf\n");
  else
    print_value ("%f\n", fmax);

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud)) 
    return (-1);

  // Apply the voxel grid
  pcl::PCLPointCloud2 output;
  compute (cloud, output, leaf_x, leaf_y, leaf_z, field, fmin, fmax);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}

