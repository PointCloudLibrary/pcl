/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <boost/filesystem.hpp> // for path, exists, ...
#include <boost/algorithm/string/case_conv.hpp> // for to_upper_copy

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int    default_k = 0;
double default_radius = 0.0;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options> [optional_arguments]\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X = use a radius of Xm around each point to determine the neighborhood (default: "); 
  print_value ("%f", default_radius); print_info (")\n");
  print_info ("                     -k X      = use a fixed number of X-nearest neighbors around each point (default: "); 
  print_value ("%f", default_k); print_info (")\n");
  print_info (" For organized datasets, an IntegralImageNormalEstimation approach will be used, with the RADIUS given value as SMOOTHING SIZE.\n");
  print_info ("\nOptional arguments are:\n");
  print_info ("                     -input_dir X  = batch process all PCD files found in input_dir\n");
  print_info ("                     -output_dir X = save the processed files from input_dir in this directory\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud,
           Eigen::Vector4f &translation, Eigen::Quaternionf &orientation)
{
  if (loadPCDFile (filename, cloud, translation, orientation) < 0)
    return (false);

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         int k, double radius)
{
  // Convert data to PointCloud<T>
  PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>);
  fromPCLPointCloud2 (*input, *xyz);

  TicToc tt;
  tt.tic ();
 
  PointCloud<Normal> normals;

  // Try our luck with organized integral image based normal estimation
  if (xyz->isOrganized ())
  {
    IntegralImageNormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud (xyz);
    ne.setNormalEstimationMethod (IntegralImageNormalEstimation<PointXYZ, Normal>::COVARIANCE_MATRIX);
    ne.setNormalSmoothingSize (float (radius));
    ne.setDepthDependentSmoothing (true);
    ne.compute (normals);
  }
  else
  {
    NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud (xyz);
    ne.setSearchMethod (search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>));
    ne.setKSearch (k);
    ne.setRadiusSearch (radius);
    ne.compute (normals);
  }

  print_highlight ("Computed normals in "); print_value ("%g", tt.toc ()); print_info (" ms for "); print_value ("%d", normals.width * normals.height); print_info (" points.\n");

  // Convert data back
  pcl::PCLPointCloud2 output_normals;
  toPCLPointCloud2 (normals, output_normals);
  concatenateFields (*input, output_normals, output);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output,
           const Eigen::Vector4f &translation, const Eigen::Quaternionf &orientation)
{
  PCDWriter w;
  w.writeBinaryCompressed (filename, output, translation, orientation);
}

int
batchProcess (const std::vector<std::string> &pcd_files, std::string &output_dir, int k, double radius)
{
#pragma omp parallel for \
  default(none) \
  shared(k, output_dir, pcd_files, radius)
  for (int i = 0; i < int (pcd_files.size ()); ++i)
  {
    // Load the first file
    Eigen::Vector4f translation;
    Eigen::Quaternionf rotation;
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    if (!loadCloud (pcd_files[i], *cloud, translation, rotation)) 
      continue;

    // Perform the feature estimation
    pcl::PCLPointCloud2 output;
    compute (cloud, output, k, radius);

    // Prepare output file name
    std::string filename = boost::filesystem::path(pcd_files[i]).filename().string();
    
    // Save into the second file
    const std::string filepath = output_dir + '/' + filename;
    saveCloud (filepath, output, translation, rotation);
  }
  return (0);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Estimate surface normals using NormalEstimation. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool batch_mode = false;

  // Command line parsing
  int k = default_k;
  double radius = default_radius;
  parse_argument (argc, argv, "-k", k);
  parse_argument (argc, argv, "-radius", radius);
  std::string input_dir, output_dir;
  if (parse_argument (argc, argv, "-input_dir", input_dir) != -1)
  {
    PCL_INFO ("Input directory given as %s. Batch process mode on.\n", input_dir.c_str ());
    if (parse_argument (argc, argv, "-output_dir", output_dir) == -1)
    {
      PCL_ERROR ("Need an output directory! Please use -output_dir to continue.\n");
      return (-1);
    }

    // Both input dir and output dir given, switch into batch processing mode
    batch_mode = true;
  }

  if (!batch_mode)
  {
    // Parse the command line arguments for .pcd files
    std::vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () != 2)
    {
      print_error ("Need one input PCD file and one output PCD file to continue.\n");
      return (-1);
    }

    print_info ("Estimating normals with a k/radius/smoothing size of: "); 
    print_value ("%d / %f / %f\n", k, radius, radius); 

    // Load the first file
    Eigen::Vector4f translation;
    Eigen::Quaternionf rotation;
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    if (!loadCloud (argv[p_file_indices[0]], *cloud, translation, rotation)) 
      return (-1);

    // Perform the feature estimation
    pcl::PCLPointCloud2 output;
    compute (cloud, output, k, radius);

    // Save into the second file
    saveCloud (argv[p_file_indices[1]], output, translation, rotation);
  }
  else
  {
    if (!input_dir.empty() && boost::filesystem::exists (input_dir))
    {
      std::vector<std::string> pcd_files;
      boost::filesystem::directory_iterator end_itr;
      for (boost::filesystem::directory_iterator itr (input_dir); itr != end_itr; ++itr)
      {
        // Only add PCD files
        if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".PCD" )
        {
          pcd_files.push_back (itr->path ().string ());
          PCL_INFO ("[Batch processing mode] Added %s for processing.\n", itr->path ().string ().c_str ());
        }
      }
      batchProcess (pcd_files, output_dir, k, radius);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
}

