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
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int    default_max_iterations = 1000;
double default_threshold = 0.05;
bool   default_negative = false;

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options> [optional_arguments]\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -thresh X = set the inlier threshold from the plane to (default: "); 
  print_value ("%g", default_threshold); print_info (")\n");
  print_info ("                     -max_it X = set the maximum number of RANSAC iterations to X (default: "); 
  print_value ("%d", default_max_iterations); print_info (")\n");
  print_info ("                     -neg 0/1  = if true (1), instead of the plane, it returns the largest cluster on top of the plane (default: "); 
  print_value ("%s", default_negative ? "true" : "false"); print_info (")\n");
  print_info ("\nOptional arguments are:\n");
  print_info ("                     -input_dir X  = batch process all PCD files found in input_dir\n");
  print_info ("                     -output_dir X = save the processed files from input_dir in this directory\n");
}

bool
loadCloud (const string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud, translation, orientation) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         int max_iterations = 1000, double threshold = 0.05, bool negative = false)
{
  // Convert data to PointCloud<T>
  PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>);
  fromPCLPointCloud2 (*input, *xyz);

  // Estimate
  TicToc tt;
  print_highlight (stderr, "Computing ");
  
  tt.tic ();

  // Refine the plane indices
  typedef SampleConsensusModelPlane<PointXYZ>::Ptr SampleConsensusModelPlanePtr;
  SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (xyz));
  RandomSampleConsensus<PointXYZ> sac (model, threshold);
  sac.setMaxIterations (max_iterations);
  bool res = sac.computeModel ();
  
  vector<int> inliers;
  sac.getInliers (inliers);
  Eigen::VectorXf coefficients;
  sac.getModelCoefficients (coefficients);

  if (!res || inliers.empty ())
  {
    PCL_ERROR ("No planar model found. Relax thresholds and continue.\n");
    return;
  }
  sac.refineModel (2, 50);
  sac.getInliers (inliers);
  sac.getModelCoefficients (coefficients);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms, plane has : "); print_value ("%lu", inliers.size ()); print_info (" points]\n");

  print_info ("Model coefficients: [");
  print_value ("%g %g %g %g", coefficients[0], coefficients[1], coefficients[2], coefficients[3]); print_info ("]\n");

  // Instead of returning the planar model as a set of inliers, return the outliers, but perform a cluster segmentation first
  if (negative)
  {
    // Remove the plane indices from the data
    PointIndices::Ptr everything_but_the_plane (new PointIndices);
    std::vector<int> indices_fullset (xyz->size ());
    for (int p_it = 0; p_it < static_cast<int> (indices_fullset.size ()); ++p_it)
      indices_fullset[p_it] = p_it;
    
    std::sort (inliers.begin (), inliers.end ());
    set_difference (indices_fullset.begin (), indices_fullset.end (),
                    inliers.begin (), inliers.end (),
                    inserter (everything_but_the_plane->indices, everything_but_the_plane->indices.begin ()));

    // Extract largest cluster minus the plane
    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setInputCloud (xyz);
    ec.setIndices (everything_but_the_plane);
    ec.extract (cluster_indices);

    // Convert data back
    copyPointCloud (*input, cluster_indices[0].indices, output);
  }
  else
  {
    // Convert data back
    PointCloud<PointXYZ> output_inliers;
    copyPointCloud (*input, inliers, output);
  }
}

void
saveCloud (const string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output, translation, orientation);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

int
batchProcess (const vector<string> &pcd_files, string &output_dir, int max_it, double thresh, bool negative)
{
  vector<string> st;
  for (size_t i = 0; i < pcd_files.size (); ++i)
  {
    // Load the first file
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    if (!loadCloud (pcd_files[i], *cloud)) 
      return (-1);

    // Perform the feature estimation
    pcl::PCLPointCloud2 output;
    compute (cloud, output, max_it, thresh, negative);

    // Prepare output file name
    string filename = pcd_files[i];
    boost::trim (filename);
    boost::split (st, filename, boost::is_any_of ("/\\"), boost::token_compress_on);
    
    // Save into the second file
    stringstream ss;
    ss << output_dir << "/" << st.at (st.size () - 1);
    saveCloud (ss.str (), output);
  }
  return (0);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Estimate the largest planar component using SACSegmentation. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool debug = false;
  console::parse_argument (argc, argv, "-debug", debug);
  if (debug)
  {
    print_highlight ("Enabling debug mode.\n");
    console::setVerbosityLevel (console::L_DEBUG);
    if (!isVerbosityLevelEnabled (L_DEBUG))
      PCL_ERROR ("Error enabling debug mode.\n");
  }

  bool batch_mode = false;

  // Command line parsing
  int max_it = default_max_iterations;
  double thresh = default_threshold;
  bool negative = default_negative;
  parse_argument (argc, argv, "-max_it", max_it);
  parse_argument (argc, argv, "-thresh", thresh);
  parse_argument (argc, argv, "-neg", negative);
  string input_dir, output_dir;
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
    vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () != 2)
    {
      print_error ("Need one input PCD file and one output PCD file to continue.\n");
      return (-1);
    }

    print_info ("Estimating planes with a threshold of: "); 
    print_value ("%g\n", thresh); 

    print_info ("Planar model segmentation: ");
    print_value ("%s\n", negative ? "false" : "true"); 

    // Load the first file
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    if (!loadCloud (argv[p_file_indices[0]], *cloud)) 
      return (-1);

    // Perform the feature estimation
    pcl::PCLPointCloud2 output;
    compute (cloud, output, max_it, thresh, negative);

    // Save into the second file
    saveCloud (argv[p_file_indices[1]], output);
  }
  else
  {
    if (input_dir != "" && boost::filesystem::exists (input_dir))
    {
      vector<string> pcd_files;
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
      batchProcess (pcd_files, output_dir, max_it, thresh, negative);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
}

