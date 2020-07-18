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

#include <pcl/segmentation/unary_classifier.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

unsigned int default_cluster_size = 10;
double default_normal_radius_search = 0.01;
double default_fpfh_radius_search = 0.05;

using PointT = PointXYZ;
using CloudT = PointCloud<PointT>;
using CloudLT = PointCloud<PointXYZRGBL>;
using FeatureT = PointCloud<FPFHSignature33>;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -label = point cloud with labeled objects \n");
  print_info ("                     -k X = k-means cluster size (default: "); 
  print_value ("%d", static_cast<int> (default_cluster_size)); print_info (")\n");
  print_info ("                     -normal-search X = Normal radius search (default: "); 
  print_value ("%f", default_normal_radius_search); print_info (")\n");
  print_info ("                     -fpfh-search X = FPFH radius search (default: "); 
  print_value ("%f", default_fpfh_radius_search); print_info (")\n");
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
compute (const CloudT::Ptr &input, std::vector<FeatureT, Eigen::aligned_allocator<FeatureT> > &output,
         unsigned int k,
         float normal_radius_search,
         float fpfh_radius_search,
         bool label)
{
  TicToc tt;
  tt.tic ();
  
  print_highlight ("Computing ");

  UnaryClassifier<PointT> classifier;
  classifier.setInputCloud (input);
  classifier.setClusterSize (k);
  classifier.setNormalRadiusSearch (normal_radius_search);
  classifier.setFPFHRadiusSearch (fpfh_radius_search);
  classifier.setLabelField (label);

  FeatureT::Ptr feature (new FeatureT);
  classifier.train (feature);  
  output.push_back (*feature);

  print_info ("[done, "); 
  print_value ("%g", tt.toc ()); 
  print_info (" ms : "); print_value ("%d", feature->width * feature->height); 
  print_info (" features]\n");
}

void
compute (const CloudLT::Ptr &input, std::vector<FeatureT, Eigen::aligned_allocator<FeatureT> > &output,
         unsigned int k,
         float normal_radius_search,
         float fpfh_radius_search,
         bool label)
{
  TicToc tt;
  tt.tic ();

  UnaryClassifier<PointXYZRGBL> classifier;
  classifier.setInputCloud (input);
  classifier.setClusterSize (k);
  classifier.setNormalRadiusSearch (normal_radius_search);
  classifier.setFPFHRadiusSearch (fpfh_radius_search);
  classifier.setLabelField (label);

  classifier.trainWithLabel (output);

  print_highlight ("Computing ");
  print_info ("[done, "); 
  print_value ("%g", tt.toc ()); 
  print_info (" ms , "); 
  print_value ("%d", output.size ()); 
  print_info (" objects : "); 
  print_value ("%d", output[0].width * output[0].height); 
  print_info (" features]\n");
}

void
saveCloud (const std::string &filename, std::vector<FeatureT, Eigen::aligned_allocator<FeatureT> > &output)
{
  TicToc tt;
  tt.tic ();

  if (output.size () == 1)
  {
    print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
    PCDWriter w;
    w.write (filename, output[0]);

    print_info ("[done, "); 
    print_value ("%g", tt.toc ()); 
    print_info (" ms : "); print_value ("%d", output[0].width * output[0].height); 
    print_info (" features]\n");    
  }
  else
  {
    for (std::size_t i = 0; i < output.size (); i++)
    {
      std::string fname (filename);
      std::string s = std::to_string(static_cast<int> (i) );
      fname += "_" + s + ".pcd";

      print_highlight ("Saving "); print_value ("%s ", fname.c_str ());

      PCDWriter w;
      w.write (fname, output[i]);

      print_info ("[done, "); 
      print_value ("%g", tt.toc ()); 
      print_info (" ms , "); 
      print_value ("%d", i); 
      print_info (" objects : "); 
      print_value ("%d", output[i].width * output[i].height); 
      print_info (" features]\n");
    }
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Train unary classifier using FPFH. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool label = (find_argument (argc, argv, "-label") != -1);

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (!label)
  {
    if (p_file_indices.size () != 2)
    {
      print_error ("Need one input PCD file and one output PCD file to continue.\n");
      return (-1);
    }
  }
  else
  {
    if (p_file_indices.size () != 1)
    {
      print_error ("Need one input PCD file and one output file name to continue.\n");
      return (-1);
    }    
  }
  
  // parse optional input arguments from the command line
  unsigned int k = default_cluster_size;
  float normal_radius_search = static_cast<float> (default_normal_radius_search);
  float fpfh_radius_search = static_cast<float> (default_fpfh_radius_search);

  parse_argument (argc, argv, "-k", k);
  parse_argument (argc, argv, "-normal-radius-search", normal_radius_search);
  parse_argument (argc, argv, "-fpfh-radius-search", fpfh_radius_search);

  print_info ("\nlabel: %d \n", label);
  print_info ("k-means cluster size: %d \n", k);
  print_info ("normal-radius-search: %f \n", normal_radius_search);
  print_info ("fpfh-radius-search: %f \n\n", fpfh_radius_search);

  std::vector<FeatureT, Eigen::aligned_allocator<FeatureT> > features;
  //FeatureT::Ptr feature (new FeatureT);
  if (!label)
  {
    // Load the input file
    CloudT::Ptr cloud (new CloudT);
    if (!loadCloud (argv[p_file_indices[0]], cloud)) 
      return (-1);
    
    // compute the features
    compute (cloud, features, k, normal_radius_search, fpfh_radius_search, label);
  }
  else
  {
    // Load the input file  
    CloudLT::Ptr cloudL (new CloudLT);
    if (!loadCloud (argv[p_file_indices[0]], cloudL)) 
      return (-1);

    // compute the features
    compute (cloudL, features, k, normal_radius_search, fpfh_radius_search, label);
  }

  // Save features to file
  saveCloud (argv[2], features);
}

