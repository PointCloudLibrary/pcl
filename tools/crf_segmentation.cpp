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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */


#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/segmentation/crf_segmentation.h>
#include <pcl/features/normal_3d.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_leaf_size = 0.005f;
double default_feature_threshold = 5.0;
double default_normal_radius_search = 0.03;

using PointT = PointXYZRGBA;
using CloudT = PointCloud<PointT>;
using CloudLT = PointCloud<PointXYZRGBL>;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -leaf x,y,z   = the VoxelGrid leaf size (default: "); 
  print_value ("%f, %f, %f", default_leaf_size, default_leaf_size, default_leaf_size); print_info (")\n");
  print_info ("                     -normal-search X = Normal radius search (default: "); 
  print_value ("%f", default_normal_radius_search); print_info (")\n");
}

bool
loadCloud (const std::string &filename, CloudT::Ptr &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, *cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->width * cloud->height); print_info (" points]\n");

  return (true);
}

bool
loadCloud (const std::string &filename, CloudLT::Ptr &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, *cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->width * cloud->height); print_info (" points]\n");

  return (true);
}

void
compute (const CloudT::Ptr &cloud, 
         const CloudLT::Ptr &anno,
         float normal_radius_search,
         float leaf_x, float leaf_y, float leaf_z,
         CloudLT::Ptr &out)
{
  TicToc tt;
  tt.tic ();
  
  print_highlight ("Computing ");

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  cloud_normals->width = cloud->width;
  cloud_normals->height = cloud->height;
  cloud_normals->points.resize (cloud->size ());
  for (std::size_t i = 0; i < cloud->size (); i++)
  {
    (*cloud_normals)[i].x = (*cloud)[i].x;
    (*cloud_normals)[i].y = (*cloud)[i].y;
    (*cloud_normals)[i].z = (*cloud)[i].z;
  }

  // estimate surface normals
  pcl::NormalEstimation<PointT, pcl::PointNormal> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setRadiusSearch (normal_radius_search);
  ne.compute (*cloud_normals);

  pcl::CrfSegmentation<PointT> crf;
  crf.setInputCloud (cloud);
  crf.setNormalCloud (cloud_normals);
  crf.setAnnotatedCloud (anno);
  crf.setVoxelGridLeafSize (leaf_x, leaf_y, leaf_z);
  crf.setSmoothnessKernelParameters (3, 3, 3, 1.0);
  crf.setAppearanceKernelParameters (30, 30, 30, 20, 20, 20, 3.5);
  crf.setSurfaceKernelParameters (20, 20, 20, 0.3f, 0.3f, 0.3f, 8.5);
  //crf.setSurfaceKernelParameters (20, 20, 20, 0.3, 0.3, 0.3, 0.0);
  crf.setNumberOfIterations (10);
  crf.segmentPoints (*out);

  print_info ("[done, "); 
  print_value ("%g", tt.toc ()); 
  print_info (" ms : "); print_value ("%d", out->width * out->height); 
  print_info (" points]\n");
}

void
saveCloud (const std::string &filename, CloudLT::Ptr &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.write (filename, *output);
  
  print_info ("[done, "); 
  print_value ("%g", tt.toc ()); print_info (" ms : "); 
  print_value ("%d", output->width * output->height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Train unary classifier using FPFH. For more information, use: %s -h\n", argv[0]);

  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 3)
  {
    print_error ("Need one input PCD file, pre-segmented PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Load the input file
  CloudT::Ptr cloud (new CloudT);
  if (!loadCloud (argv[p_file_indices[0]], cloud)) 
    return (-1);

  // Load the input file
  CloudLT::Ptr cloud_anno (new CloudLT);
  if (!loadCloud (argv[p_file_indices[1]], cloud_anno)) 
    return (-1);


  // TODO:: make this as an optional argument ??
  pcl::Indices tmp_indices;
  pcl::removeNaNFromPointCloud (*cloud, *cloud, tmp_indices);
  
  // parse optional input arguments from the command line
  float normal_radius_search = static_cast<float> (default_normal_radius_search);

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
    print_error ("Leaf size must be specified with either 1 or 3 numbers (%lu given). ", values.size ());
  }
  print_info ("Using a leaf size of: "); print_value ("%f, %f, %f\n", leaf_x, leaf_y, leaf_z);

  parse_argument (argc, argv, "-normal-radius-search", normal_radius_search);

  // segment the pre-segmented cloud
  CloudLT::Ptr out (new CloudLT);
  compute (cloud, cloud_anno, normal_radius_search, leaf_x, leaf_y, leaf_z, out);

  // save cloud
  saveCloud (argv[p_file_indices[2]], out);
}

