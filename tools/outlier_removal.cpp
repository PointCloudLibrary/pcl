/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

std::string default_method = "radius";

int default_mean_k = 2;
double default_std_dev_mul = 0.0;
int default_negative = 0;

double default_radius = 0.0;
int default_min_pts = 0;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -method X = the outlier removal method to be used (options: radius / statistical) (default: ");
  print_value ("%s", default_method.c_str ()); print_info (")\n");
  print_info ("                     -radius X = (RadiusOutlierRemoval) the sphere radius used for determining the k-nearest neighbors (default: ");
  print_value ("%d", default_min_pts); print_info (")\n");
  print_info ("                     -min_pts X = (RadiusOutlierRemoval) the minimum number of neighbors that a point needs to have in the given search radius in order to be considered an inlier (default: ");
  print_value ("%d", default_min_pts); print_info (")\n");
  print_info ("                     -mean_k X = (StatisticalOutlierRemoval only) the number of points to use for mean distance estimation (default: ");
  print_value ("%d", default_mean_k); print_info (")\n");
  print_info ("                     -std_dev_mul X = (StatisticalOutlierRemoval only) the standard deviation multiplier threshold (default: ");
  print_value ("%f", default_std_dev_mul); print_info (")\n\n");
  print_info ("                     -negative X = decides whether the inliers should be returned (1), or the outliers (0). (default: ");
  print_value ("%d", default_negative); print_info (")\n");
  print_info ("                     -keep_organized = keep the filtered points in organized format.\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud,
           Eigen::Vector4f &translation, Eigen::Quaternionf &orientation)
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
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         const std::string& method,
         int min_pts, double radius,
         int mean_k, double std_dev_mul, bool negative, bool keep_organized)
{

  PointCloud<PointXYZ>::Ptr xyz_cloud_pre (new pcl::PointCloud<PointXYZ> ()),
                            xyz_cloud (new pcl::PointCloud<PointXYZ> ());
  fromPCLPointCloud2 (*input, *xyz_cloud_pre);

  pcl::PointIndices::Ptr removed_indices (new PointIndices),
                         indices (new PointIndices);
  pcl::Indices valid_indices;
  if (keep_organized)
  {
    xyz_cloud = xyz_cloud_pre;
    for (int i = 0; i < int (xyz_cloud->size ()); ++i)
      valid_indices.push_back (i);
  }
  else
    removeNaNFromPointCloud<PointXYZ> (*xyz_cloud_pre, *xyz_cloud, valid_indices);

  TicToc tt;
  tt.tic ();
  PointCloud<PointXYZ>::Ptr xyz_cloud_filtered (new PointCloud<PointXYZ> ());
  if (method == "statistical")
  {
    StatisticalOutlierRemoval<PointXYZ> filter (true);
    filter.setInputCloud (xyz_cloud);
    filter.setMeanK (mean_k);
    filter.setStddevMulThresh (std_dev_mul);
    filter.setNegative (negative);
    filter.setKeepOrganized (keep_organized);
    PCL_INFO ("Computing filtered cloud from %lu points with mean_k %d, std_dev_mul %f, inliers %d ...", xyz_cloud->size (), filter.getMeanK (), filter.getStddevMulThresh (), filter.getNegative ());
    filter.filter (*xyz_cloud_filtered);
    // Get the indices that have been explicitly removed
    filter.getRemovedIndices (*removed_indices);
  }
  else if (method == "radius")
  {
    RadiusOutlierRemoval<PointXYZ> filter (true);
    filter.setInputCloud (xyz_cloud);
    filter.setRadiusSearch (radius);
    filter.setMinNeighborsInRadius (min_pts);
    filter.setNegative (negative);
    filter.setKeepOrganized (keep_organized);
    PCL_INFO ("Computing filtered cloud from %lu points with radius %f, min_pts %d ...", xyz_cloud->size (), radius, min_pts);
    filter.filter (*xyz_cloud_filtered);
    // Get the indices that have been explicitly removed
    filter.getRemovedIndices (*removed_indices);
  }
  else
  {
    PCL_ERROR ("%s is not a valid filter name! Quitting!\n", method.c_str ());
    return;
  }
    
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", xyz_cloud_filtered->width * xyz_cloud_filtered->height); print_info (" points, %lu indices removed]\n", removed_indices->indices.size ());

  if (keep_organized)
  {
    pcl::PCLPointCloud2 output_filtered;
    toPCLPointCloud2 (*xyz_cloud_filtered, output_filtered);
    concatenateFields (*input, output_filtered, output);
  }
  else 
  {
    // Make sure we are addressing values in the original index vector
    for (const auto& i : (removed_indices->indices))
      indices->indices.push_back (valid_indices[i]);

    // Extract the indices of the remaining points
    pcl::ExtractIndices<pcl::PCLPointCloud2> ei;
    ei.setInputCloud (input);
    ei.setIndices (indices);
    ei.setNegative (true);
    ei.filter (output);
  }
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output,
           const Eigen::Vector4f &translation, const Eigen::Quaternionf &rotation)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output, translation, rotation);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Statistical Outlier Removal filtering of a point cloud. For more information, use: %s -h\n", argv[0]);

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
  std::string method = default_method;
  int min_pts = default_min_pts;
  double radius = default_radius;
  int mean_k = default_mean_k;
  double std_dev_mul = default_std_dev_mul;
  int negative = default_negative;
  

  parse_argument (argc, argv, "-method", method);
  parse_argument (argc, argv, "-radius", radius);
  parse_argument (argc, argv, "-min_pts", min_pts);
  parse_argument (argc, argv, "-mean_k", mean_k);
  parse_argument (argc, argv, "-std_dev_mul", std_dev_mul);
  parse_argument (argc, argv, "-negative", negative);
  bool keep_organized = find_switch (argc, argv, "-keep_organized");

  // Load the first file
  Eigen::Vector4f translation;
  Eigen::Quaternionf rotation;
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud, translation, rotation))
    return (-1);
  
  if (keep_organized && cloud->height == 1)
  {
    print_error ("Point cloud dataset (%s) is not organized (height = %d), but -keep_organized requested!\n", argv[p_file_indices[0]], cloud->height);
    return (-1);
  }

  // Do the smoothing
  pcl::PCLPointCloud2 output;
  compute (cloud, output, method, min_pts, radius, mean_k, std_dev_mul, negative, keep_organized);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output, translation, rotation);
}
