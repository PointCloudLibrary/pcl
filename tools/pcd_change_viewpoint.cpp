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
 * $Id: normal_estimation.cpp 4990 2012-03-09 08:30:36Z rusu $
 *
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -viewpoint Tx,Ty,Tz,Qw,Qx,Qy,Qz\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  Eigen::Vector4f    translation;
  Eigen::Quaternionf orientation;

  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud, translation, orientation) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("VIEWPOINT information is: "); 
  print_value ("%.2f %.2f %.2f / %.2f %.2f %.2f %.2f\n", 
      translation.x (), translation.y (), translation.z (),
      orientation.w (), orientation.x (), orientation.y (), orientation.z ());

  return (true);
}

void
saveCloud (const std::string &filename, 
           const Eigen::Vector4f    &translation,
           const Eigen::Quaternionf &orientation,
           const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output, translation, orientation);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Change viewpoint information in a PCD file. For more information, use: %s -h\n", argv[0]);

  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  }

  Eigen::Vector4f    translation = Eigen::Vector4f::Zero ();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity ();

  std::vector<float> values;
  if (parse_x_arguments (argc, argv, "-viewpoint", values) > -1)
  {
    if (values.size () == 7)
    {
      translation = Eigen::Vector4f (values[0], values[1], values[2], 0.0f);
      orientation = Eigen::Quaternionf (values[3], values[4], values[5], values[6]);
    }
    else
    {
      print_error ("Wrong number of values given (%lu): ", values.size ());
      print_error ("The VIEWPOINT specified with -viewpoint must contain 7 elements (tx, ty, tz, qw, qx, qy, qz).\n");
      return (-1);
    }
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Load the first file
  pcl::PCLPointCloud2 cloud;
  if (!loadCloud (argv[p_file_indices[0]], cloud)) 
    return (-1);

  print_info ("Saving output PCD file with the following VIEWPOINT information: "); 
  print_value ("%.2f %.2f %.2f / %.2f %.2f %.2f %.2f\n", 
      translation.x (), translation.y (), translation.z (),
      orientation.w (), orientation.x (), orientation.y (), orientation.z ());

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], translation, orientation, cloud);
}

