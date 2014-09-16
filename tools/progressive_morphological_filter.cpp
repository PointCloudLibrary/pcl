/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;
typedef const Cloud::ConstPtr ConstCloudPtr;

int default_max_window_size = 33;
float default_slope = 0.7f;
float default_max_distance = 10.0f;
float default_initial_distance = 0.15f;
float default_cell_size = 1.0f;
float default_base = 2.0f;
bool default_exponential = true;
int default_verbosity_level = 3;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -max_window_size X = maximum window size (default: ");
  print_value ("%d", default_max_window_size); print_info (")\n");
  print_info ("                     -slope X = slope value to compute threshold (default: ");
  print_value ("%f", default_slope); print_info (")\n");
  print_info ("                     -max_distnace X = maximum distance from parameterized ground surface to be considered ground (default: ");
  print_value ("%f", default_max_distance); print_info (")\n");
  print_info ("                     -initial_distance X = initial distance from parameterized ground surface to be considered ground (default: ");
  print_value ("%f", default_initial_distance); print_info (")\n");
  print_info ("                     -cell_size X = cell size (default: ");
  print_value ("%f", default_cell_size); print_info (")\n");
  print_info ("                     -base X = base to be used in computing progressive window sizes (default: ");
  print_value ("%f", default_base); print_info (")\n");
  print_info ("                     -exponential X = use exponential growth? (default: ");
  print_value ("%s", default_exponential?"true":"false"); print_info (")\n");
  print_info ("                     -approximate X = use approximate? (default: false\n");
  print_info ("                     -input_dir X  = batch process all PCD files found in input_dir\n");
  print_info ("                     -output_dir X = save the processed files from input_dir in this directory\n");
  print_info ("                     -verbosity X = verbosity level (default: ");
  print_value ("%d", default_verbosity_level); print_info (")\n");
}

bool
loadCloud (const std::string &filename, Cloud &cloud)
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

void
compute (ConstCloudPtr &input, Cloud &output, int max_window_size, float slope, float max_distance, float initial_distance, float cell_size, float base, bool exponential, bool approximate)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  std::vector<int> ground;

  if (approximate)
  {
    PCL_DEBUG ("approx with %d points\n", input->points.size ());
    ApproximateProgressiveMorphologicalFilter<PointType> pmf;
    pmf.setInputCloud (input);
    pmf.setMaxWindowSize (max_window_size);
    pmf.setSlope (slope);
    pmf.setMaxDistance (max_distance);
    pmf.setInitialDistance (initial_distance);
    pmf.setCellSize (cell_size);
    pmf.setBase (base);
    pmf.setExponential (exponential);
    pmf.extract (ground);
  }
  else
  {
    PCL_DEBUG ("full\n");
    ProgressiveMorphologicalFilter<PointType> pmf;
    pmf.setInputCloud (input);
    pmf.setMaxWindowSize (max_window_size);
    pmf.setSlope (slope);
    pmf.setMaxDistance (max_distance);
    pmf.setInitialDistance (initial_distance);
    pmf.setCellSize (cell_size);
    pmf.setBase (base);
    pmf.setExponential (exponential);
    pmf.extract (ground);
  }

  PointIndicesPtr idx (new PointIndices);
  idx->indices = ground;

  ExtractIndices<PointType> extract;
  extract.setInputCloud (input);
  extract.setIndices (idx);
  extract.setNegative (false);
  extract.filter (output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

void
saveCloud (const std::string &filename, const Cloud &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

int
batchProcess (const vector<string> &pcd_files, string &output_dir, int max_window_size, float slope, float max_distance, float initial_distance, float cell_size, float base, bool exponential, bool approximate)
{
  vector<string> st;
  for (size_t i = 0; i < pcd_files.size (); ++i)
  {
    // Load the first file
    Cloud::Ptr cloud (new Cloud);
    if (!loadCloud (pcd_files[i], *cloud))
      return (-1);

    // Perform the feature estimation
    Cloud output;
    compute (cloud, output, max_window_size, slope, max_distance, initial_distance, cell_size, base, exponential, approximate);

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
  print_info ("Filter a point cloud using the pcl::ProgressiveMorphologicalFilter. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool batch_mode = false;

  // Command line parsing
  int max_window_size = default_max_window_size;
  float slope = default_slope;
  float max_distance = default_max_distance;
  float initial_distance = default_initial_distance;
  float cell_size = default_cell_size;
  float base = default_base;
  bool exponential = default_exponential;
  bool approximate;
  int verbosity_level = default_verbosity_level;
  parse_argument (argc, argv, "-max_window_size", max_window_size);
  parse_argument (argc, argv, "-slope", slope);
  parse_argument (argc, argv, "-max_distance", max_distance);
  parse_argument (argc, argv, "-initial_distance", initial_distance);
  parse_argument (argc, argv, "-cell_size", cell_size);
  parse_argument (argc, argv, "-base", base);
  parse_argument (argc, argv, "-exponential", exponential);
  approximate = find_switch (argc, argv, "-approximate");
  parse_argument (argc, argv, "-verbosity", verbosity_level);
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

  switch (verbosity_level)
  {
    case 0:
      pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
      break;

    case 1:
      pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
      break;

    case 2:
      pcl::console::setVerbosityLevel(pcl::console::L_WARN);
      break;

    case 3:
      pcl::console::setVerbosityLevel(pcl::console::L_INFO);
      break;

    case 4:
      pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
      break;

    default:
      pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
      break;
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

    // Load the first file
    Cloud::Ptr cloud (new Cloud);
    if (!loadCloud (argv[p_file_indices[0]], *cloud))
      return (-1);

    // Perform the feature estimation
    Cloud output;
    compute (cloud, output, max_window_size, slope, max_distance, initial_distance, cell_size, base, exponential, approximate);

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
      batchProcess (pcd_files, output_dir, max_window_size, slope, max_distance, initial_distance, cell_size, base, exponential, approximate);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
}

