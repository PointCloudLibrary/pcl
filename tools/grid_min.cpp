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
#include <pcl/filters/grid_minimum.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;
typedef const Cloud::ConstPtr ConstCloudPtr;

float default_resolution = 1.0f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -resolution X = xy resolution of the grid (default: ");
  print_value ("%f", default_resolution); print_info (")\n");
  print_info ("                     -input_dir X  = batch process all PCD files found in input_dir\n");
  print_info ("                     -output_dir X = save the processed files from input_dir in this directory\n");
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
compute (ConstCloudPtr &input, Cloud &output, float resolution)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  GridMinimum<PointType> gm (resolution);
  gm.setInputCloud (input);
  gm.filter (output);

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
batchProcess (const vector<string> &pcd_files, string &output_dir,
              float resolution)
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
    compute (cloud, output, resolution);

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
  print_info ("Filter a point cloud using the pcl::GridMinimum filter. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool batch_mode = false;

  // Command line parsing
  float resolution = default_resolution;
  parse_argument (argc, argv, "-resolution", resolution);
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
    compute (cloud, output, resolution);

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
      batchProcess (pcd_files, output_dir, resolution);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
}
