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
 *
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

std::string default_feature_name = "FPFHEstimation";
int    default_n_k = 0;
double default_n_radius = 0.0;
int    default_f_k = 0;
double default_f_radius = 0.0;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -feature X = the feature descriptor algorithm to be used (default: ");
  print_value ("%s", default_feature_name.c_str ()); print_info (")\n");
  print_info ("                     -n_radius X = use a radius of Xm around each point to determine the neighborhood in normal estimation (default: ");
  print_value ("%f", default_n_radius); print_info (")\n");
  print_info ("                     -n_k X      = use a fixed number of X-nearest neighbors around each point in normal estimation (default: ");
  print_value ("%f", default_n_k); print_info (")\n");
  print_info ("                     -f_radius X = use a radius of Xm around each point to determine the neighborhood in feature extraction (default: ");
  print_value ("%f", default_f_radius); print_info (")\n");
  print_info ("                     -f_k X      = use a fixed number of X-nearest neighbors around each point in feature extraction(default: ");
  print_value ("%f", default_f_k); print_info (")\n");
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

template <typename FeatureAlgorithm, typename PointIn, typename NormalT, typename PointOut>
void
computeFeatureViaNormals (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         int argc, char** argv, bool set_search_flag = true)
{
  int n_k = default_n_k;
  int f_k = default_f_k;
  double n_radius = default_n_radius;
  double f_radius = default_f_radius;
  parse_argument (argc, argv, "-n_k", n_k);
  parse_argument (argc, argv, "-n_radius", n_radius);
  parse_argument (argc, argv, "-f_k", f_k);
  parse_argument (argc, argv, "-f_radius", f_radius);

  // Convert data to PointCloud<PointIn>
  typename PointCloud<PointIn>::Ptr xyz (new PointCloud<PointIn>);
  fromPCLPointCloud2 (*input, *xyz);

  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  NormalEstimation<PointIn, NormalT> ne;
  ne.setInputCloud (xyz);
  ne.setSearchMethod (typename pcl::search::KdTree<PointIn>::Ptr (new pcl::search::KdTree<PointIn>));
  ne.setKSearch (n_k);
  ne.setRadiusSearch (n_radius);

  typename PointCloud<NormalT>::Ptr normals = typename PointCloud<NormalT>::Ptr (new PointCloud<NormalT>);
  ne.compute (*normals);

  FeatureAlgorithm feature_est;
  feature_est.setInputCloud (xyz);
  feature_est.setInputNormals (normals);

  feature_est.setSearchMethod (typename pcl::search::KdTree<PointIn>::Ptr (new pcl::search::KdTree<PointIn>));

  PointCloud<PointOut> output_features;

  if (set_search_flag) {
    feature_est.setKSearch (f_k);
    feature_est.setRadiusSearch (f_radius);
  }

  feature_est.compute (output_features);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");

  // Convert data back
  toPCLPointCloud2 (output_features, output);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFile (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Extract features from a point cloud. For more information, use: %s -h\n", argv[0]);

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
  std::string feature_name = default_feature_name;
  parse_argument (argc, argv, "-feature", feature_name);

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud))
    return (-1);

  // Perform the feature estimation
  pcl::PCLPointCloud2 output;
  if (feature_name == "PFHEstimation") 
    computeFeatureViaNormals< PFHEstimation<PointXYZ, Normal, PFHSignature125>, PointXYZ, Normal, PFHSignature125>
      (cloud, output, argc, argv);
  else if (feature_name == "FPFHEstimation")
    computeFeatureViaNormals< FPFHEstimation<PointXYZ, Normal, FPFHSignature33>, PointXYZ, Normal, FPFHSignature33>
      (cloud, output, argc, argv);
  else if (feature_name == "VFHEstimation")
    computeFeatureViaNormals< VFHEstimation<PointXYZ, Normal, VFHSignature308>, PointXYZ, Normal, VFHSignature308>
    (cloud, output, argc, argv, false);
  else
  {
    print_error ("Valid feature names are PFHEstimation, FPFHEstimation, VFHEstimation.\n");
    return (-1);
  }

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}
