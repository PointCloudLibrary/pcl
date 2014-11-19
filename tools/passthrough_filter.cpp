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
 * $Id$
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>


using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_min = 0.0f,
      default_max = 1.0f;
bool default_inside = true;
bool default_keep_organized = true;
std::string default_field_name = "z";

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -field X = the field of the point cloud we want to apply the filter to (default: ");
  print_value ("%s", default_field_name.c_str ()); print_info (")\n");
  print_info ("                     -min X = lower limit of the filter (default: ");
  print_value ("%f", default_min); print_info (")\n");
  print_info ("                     -max X = upper limit of the filter (default: ");
  print_value ("%f", default_max); print_info (")\n");
  print_info ("                     -inside X = keep the points inside the [min, max] interval or not (default: ");
  print_value ("%d", default_inside); print_info (")\n");
  print_info ("                     -keep 0/1 = keep the points organized (1) or not (default: ");
  print_value ("%d", default_keep_organized); print_info (")\n");
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

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         std::string field_name, float min, float max, bool inside, bool keep_organized)
{
  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  PassThrough<pcl::PCLPointCloud2> passthrough_filter;
  passthrough_filter.setInputCloud (input);
  passthrough_filter.setFilterFieldName (field_name);
  passthrough_filter.setFilterLimits (min, max);
  passthrough_filter.setFilterLimitsNegative (!inside);
  passthrough_filter.setKeepOrganized (keep_organized);
  passthrough_filter.filter (output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
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
              std::string field_name, float min, float max, bool inside, bool keep_organized)
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
    compute (cloud, output, field_name, min, max, inside, keep_organized);

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
  print_info ("Filter a point cloud using the pcl::PassThroughFilterEstimate. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool batch_mode = false;

  // Command line parsing
  float min = default_min, max = default_max;
  bool inside = default_inside;
  bool keep_organized = default_keep_organized;
  std::string field_name = default_field_name;
  parse_argument (argc, argv, "-min", min);
  parse_argument (argc, argv, "-max", max);
  parse_argument (argc, argv, "-inside", inside);
  parse_argument (argc, argv, "-field", field_name);
  parse_argument (argc, argv, "-keep", keep_organized);
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
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    if (!loadCloud (argv[p_file_indices[0]], *cloud))
      return (-1);

    // Perform the feature estimation
    pcl::PCLPointCloud2 output;
    compute (cloud, output, field_name, min, max, inside, keep_organized);

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
      batchProcess (pcd_files, output_dir, field_name, min, max, inside, keep_organized);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
}
