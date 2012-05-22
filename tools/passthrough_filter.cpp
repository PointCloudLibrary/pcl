/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: passthrough_filter.cpp 2417 2011-09-07 07:22:29Z rusu $
 */

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_min = 0.0f,
      default_max = 1.0f;
bool default_inside = true;
bool default_keep_organized = true;
std::string default_field_name = "z";

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -field X = the field of the point cloud we want to apply the filter to (default: ");
  print_value ("%s", default_field_name.c_str ()); print_info (")\n");
  print_info ("                     -min X = lower limit of the filter (default: ");
  print_value ("%f", default_min); print_info (")\n");
  print_info ("                     -max X = upper limit of the filter (default: ");
  print_value ("%f", default_max); print_info (")\n");
  print_info ("                     -inside X = keep the points inside the [min, max] interval or not (default: ");
  print_value ("%d", default_inside); print_info (")\n");
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
         std::string field_name, float min, float max, bool inside, bool keep_organized)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  PassThrough<sensor_msgs::PointCloud2> passthrough_filter;
  passthrough_filter.setInputCloud (input);
  passthrough_filter.setFilterFieldName (field_name);
  passthrough_filter.setFilterLimits (min, max);
  passthrough_filter.setFilterLimitsNegative (!inside);
  passthrough_filter.setKeepOrganized (keep_organized);
  passthrough_filter.filter (output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

void
saveCloud (const std::string &filename, const sensor_msgs::PointCloud2 &output)
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
  print_info ("Filter a point cloud using the pcl::PassThroughFilterEstimate. For more information, use: %s -h\n", argv[0]);

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
  float min = default_min, max = default_max;
  bool inside = default_inside;
  bool keep_organized = default_keep_organized;
  std::string field_name = default_field_name;
  parse_argument (argc, argv, "-min", min);
  parse_argument (argc, argv, "-max", max);
  parse_argument (argc, argv, "-inside", inside);
  parse_argument (argc, argv, "-field", field_name);
  parse_argument (argc, argv, "-keep", keep_organized);

  // Load the first file
  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud))
    return (-1);

  // Perform the feature estimation
  sensor_msgs::PointCloud2 output;
  compute (cloud, output, field_name, min, max, inside, keep_organized);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}
