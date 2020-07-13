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
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/sample_consensus/sac_model_plane.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd A B C D\n", argv[0]);
  print_info ("  where the plane is represented by the following equation:\n");
  print_info ("                     Ax + By + Cz + D = 0\n"); 
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud, translation, orientation) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
project (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output, float a, float b, float c, float d)
{
  Eigen::Vector4f coeffs;
  coeffs << a, b, c, d;

  // Convert data to PointCloud<T>
  PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>);
  fromPCLPointCloud2 (*input, *xyz);

  // Estimate
  TicToc tt;
  tt.tic ();

  //First, we'll find a point on the plane
  print_highlight (stderr, "Projecting ");

  PointCloud<PointXYZ>::Ptr projected_cloud_pcl (new PointCloud<PointXYZ>);
  projected_cloud_pcl->width = xyz->width;
  projected_cloud_pcl->height = xyz->height;
  projected_cloud_pcl->is_dense = xyz->is_dense;
  projected_cloud_pcl->sensor_origin_ = xyz->sensor_origin_;
  projected_cloud_pcl->sensor_orientation_ = xyz->sensor_orientation_;

  for (const auto& point: *xyz)
  {
    pcl::PointXYZ projection;
    pcl::projectPoint<PointXYZ> (point, coeffs, projection);
    projected_cloud_pcl->points.push_back(projection);
  }


  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : ");
  pcl::io::savePCDFile ("foo.pcd", *projected_cloud_pcl);

  // Convert data back
  pcl::PCLPointCloud2 projected_cloud;
  toPCLPointCloud2 (*projected_cloud_pcl, projected_cloud);

  //we can actually use concatenate fields to inject our projection into the
  //output, the second argument overwrites the first's fields for those that
  //are shared
  concatenateFields (*input, projected_cloud, output);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::io::savePCDFile (filename, output, translation, orientation, false);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Estimate surface normals using pcl::NormalEstimation. For more information, use: %s -h\n", argv[0]);

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

  if(argc != 7)
  {
    print_error("This function takes: input_file output_file A B C D");
    return(-1);
  }

  // Command line parsing
  float a = static_cast<float> (atof (argv[3]));
  float b = static_cast<float> (atof (argv[4]));
  float c = static_cast<float> (atof (argv[5]));
  float d = static_cast<float> (atof (argv[6]));

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud)) 
    return (-1);

  // Perform the feature estimation
  pcl::PCLPointCloud2 output;
  project (cloud, output, a, b, c, d);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}

