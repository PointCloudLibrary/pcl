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

double default_feature_threshold = 5.0;
double default_normal_radius_search = 0.01;
double default_fpfh_radius_search = 0.05;

typedef PointXYZRGBA PointT;
typedef PointCloud<PointT> CloudT;
typedef PointCloud<PointXYZRGBL> CloudLT;
typedef PointCloud<FPFHSignature33> FeatureT;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -d = trained features directory \n");
  print_info ("                     -threshold X = feature threshold (default: "); 
  print_value ("%f", default_feature_threshold); print_info (")\n");
  print_info ("                     -normal-search X = Normal radius search (default: "); 
  print_value ("%f", default_normal_radius_search); print_info (")\n");
  print_info ("                     -fpfh-search X = FPFH radius search (default: "); 
  print_value ("%f", default_fpfh_radius_search); print_info (")\n");
}

bool
loadTrainedFeatures (std::vector<FeatureT::Ptr> &out,
                     const boost::filesystem::path &base_dir)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return false;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {    
    if (!boost::filesystem::is_directory (it->status ()) &&
        boost::filesystem::extension (it->path ()) == ".pcd")
    {   
      std::stringstream ss;
      ss << it->path ().string ();

      print_highlight ("Loading %s \n", ss.str ().c_str ());
      
      FeatureT::Ptr features (new FeatureT);
      if (loadPCDFile (ss.str ().c_str (), *features) < 0)
        return false;

      out.push_back (features);
    }
  }
  return true;
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

void
compute (const CloudT::Ptr &input, std::vector<FeatureT::Ptr> &trained_features,
         CloudLT::Ptr &out,
         float normal_radius_search,
         float fpfh_radius_search,
         float feature_threshold)
{
  TicToc tt;
  tt.tic ();
  
  print_highlight ("Computing ");

  UnaryClassifier<PointT> classifier;
  classifier.setInputCloud (input);
  classifier.setTrainedFeatures (trained_features);
  classifier.setNormalRadiusSearch (normal_radius_search);
  classifier.setFPFHRadiusSearch (fpfh_radius_search);
  classifier.setFeatureThreshold (feature_threshold);

  classifier.segment (out);

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
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Load the input file
  CloudT::Ptr cloud (new CloudT);
  if (!loadCloud (argv[p_file_indices[0]], cloud)) 
    return (-1);

  // TODO:: make this as an optional argument ??
  std::vector<int> tmp_indices;
  pcl::removeNaNFromPointCloud (*cloud, *cloud, tmp_indices);
  
  // parse optional input arguments from the command line
  float normal_radius_search = static_cast<float> (default_normal_radius_search);
  float fpfh_radius_search = static_cast<float> (default_fpfh_radius_search);
  float feature_threshold = static_cast<float> (default_feature_threshold);
  std::string dir_name;

  parse_argument (argc, argv, "-d", dir_name);
  parse_argument (argc, argv, "-threshold", feature_threshold);
  parse_argument (argc, argv, "-normal-radius-search", normal_radius_search);
  parse_argument (argc, argv, "-fpfh-radius-search", fpfh_radius_search);


  print_info ("trained feature directory: %s \n", dir_name.c_str ());

  // load the trained features
  std::vector<FeatureT::Ptr> trained_features;
  if (!loadTrainedFeatures (trained_features, dir_name.c_str ()))
    return (-1);

  print_info ("feature_threshold: %f \n", feature_threshold);
  print_info ("normal-radius-search: %f \n", normal_radius_search);
  print_info ("fpfh-radius-search: %f \n\n", fpfh_radius_search);

  CloudLT::Ptr out (new CloudLT);
  compute (cloud, trained_features, out, normal_radius_search, fpfh_radius_search, feature_threshold);

  saveCloud (argv[p_file_indices[1]], out);
}

