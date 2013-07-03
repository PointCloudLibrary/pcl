/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <vector>
#include "boost.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int    default_min = 100;
int    default_max = 25000;
double default_tolerance = 0.02;

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -min X = use a minimum of X points peer cluster (default: "); 
  print_value ("%d", default_min); print_info (")\n");
  print_info ("                     -max X      = use a maximum of X points peer cluster (default: "); 
  print_value ("%d", default_max); print_info (")\n");
  print_info ("                     -tolerance X = the spacial distance between clusters (default: "); 
  print_value ("%lf", default_tolerance); print_info (")\n");
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
compute (const pcl::PCLPointCloud2::ConstPtr &input, std::vector<pcl::PCLPointCloud2::Ptr> &output,
         int min, int max, double tolerance)
{
  // Convert data to PointCloud<T>
  PointCloud<pcl::PointXYZ>::Ptr xyz (new PointCloud<pcl::PointXYZ>);
  fromPCLPointCloud2 (*input, *xyz);

  // Estimate
  TicToc tt;
  tt.tic ();
  
  print_highlight (stderr, "Computing ");

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (xyz);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance);
  ec.setMinClusterSize (min);
  ec.setMaxClusterSize (max);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyz);
  ec.extract (cluster_indices);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cluster_indices.size ()); print_info (" clusters]\n");

  output.reserve (cluster_indices.size ());
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud (input);
    extract.setIndices (boost::make_shared<const pcl::PointIndices> (*it));
    pcl::PCLPointCloud2::Ptr out (new pcl::PCLPointCloud2);
    extract.filter (*out);
    output.push_back (out);
  }
}

void
saveCloud (const std::string &filename, const std::vector<pcl::PCLPointCloud2::Ptr> &output)
{
  TicToc tt;
  tt.tic ();

  std::string basename = filename.substr (0, filename.length () - 4);

  for (size_t i = 0; i < output.size (); i++)
  {
    std::string clustername = basename + boost::lexical_cast<std::string>(i) + ".pcd";
    print_highlight ("Saving "); print_value ("%s ", clustername.c_str ());

    pcl::io::savePCDFile (clustername, *(output[i]), translation, orientation, false);

    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output[i]->width * output[i]->height); print_info (" points]\n");
  }
  
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Extract point clusters using pcl::EuclideanClusterExtraction. For more information, use: %s -h\n", argv[0]);
  bool help = false;
  parse_argument (argc, argv, "-h", help);
  if (argc < 3 || help)
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
  int min = default_min;
  int max = default_max;
  double tolerance = default_tolerance;
  parse_argument (argc, argv, "-min", min);
  parse_argument (argc, argv, "-max", max);
  parse_argument (argc, argv, "-tolerance", tolerance);

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud)) 
    return (-1);

  // Perform the feature estimation
  std::vector<pcl::PCLPointCloud2::Ptr> output;
  compute (cloud, output, min, max, tolerance);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}
