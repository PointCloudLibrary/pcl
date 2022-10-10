/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/conditional_removal.h>
#include <boost/filesystem.hpp> // for path, exists, ...
#include <boost/algorithm/string/case_conv.hpp> // for to_upper_copy


using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;

float default_radius = 1.0f;
bool default_inside = true;
bool default_keep_organized = true;

void
printHelp (int, char **argv)
{
  pcl::console::print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  pcl::console::print_info ("  where options are:\n");
  pcl::console::print_info ("                     -radius X = Radius of the spere to filter (default: ");
  pcl::console::print_value ("%s", default_radius); pcl::console::print_info (")\n");
  pcl::console::print_info ("                     -inside X = keep the points inside the [min, max] interval or not (default: ");
  pcl::console::print_value ("%d", default_inside); pcl::console::print_info (")\n");
  pcl::console::print_info ("                     -keep 0/1 = keep the points organized (1) or not (default: ");
  pcl::console::print_value ("%d", default_keep_organized); pcl::console::print_info (")\n");
}

bool
loadCloud (const std::string &filename, Cloud::Ptr cloud)
{
  pcl::console::TicToc tt;
  pcl::console::print_highlight ("Loading "); pcl::console::print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (pcl::io::loadPCDFile (filename, *cloud) < 0)
    return (false);
  pcl::console::print_info ("[done, "); pcl::console::print_value ("%g", tt.toc ()); pcl::console::print_info (" ms : "); pcl::console::print_value ("%d", cloud->size ()); pcl::console::print_info (" points]\n");

  return (true);
}

void
compute (const Cloud::Ptr &input, Cloud::Ptr &output,
         float radius, bool inside, bool keep_organized)
{
  // Estimate
  pcl::console::TicToc tt;
  tt.tic ();

  pcl::console::print_highlight (stderr, "Computing ");

  pcl::ConditionOr<PointType>::Ptr cond (new pcl::ConditionOr<PointType> ());
  cond->addComparison (pcl::TfQuadraticXYZComparison<PointType>::ConstPtr (new pcl::TfQuadraticXYZComparison<PointType> (inside ? pcl::ComparisonOps::LT : pcl::ComparisonOps::GT, Eigen::Matrix3f::Identity (),
                                                                                                                  Eigen::Vector3f::Zero (), - radius * radius)));

  pcl::ConditionalRemoval<PointType> condrem;
  condrem.setCondition (cond);
  condrem.setInputCloud (input);
  condrem.setKeepOrganized (keep_organized);
  condrem.filter (*output);

  pcl::console::print_info ("[done, "); pcl::console::print_value ("%g", tt.toc ()); pcl::console::print_info (" ms : "); pcl::console::print_value ("%d", output->size ()); pcl::console::print_info (" points]\n");
}

void
saveCloud (const std::string &filename, const Cloud::Ptr &output)
{
  pcl::console::TicToc tt;
  tt.tic ();

  pcl::console::print_highlight ("Saving "); pcl::console::print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFileBinaryCompressed (filename, *output);

  pcl::console::print_info ("[done, "); pcl::console::print_value ("%g", tt.toc ()); pcl::console::print_info (" ms : "); pcl::console::print_value ("%d", output->size ()); pcl::console::print_info (" points]\n");
}

int
batchProcess (const std::vector<std::string> &pcd_files, std::string &output_dir,
              float radius, bool inside, bool keep_organized)
{
  for (const auto &pcd_file : pcd_files)
  {
    // Load the first file
    Cloud::Ptr cloud (new Cloud);
    if (!loadCloud (pcd_file, cloud))
      return (-1);

    // Perform the feature estimation
    Cloud::Ptr output (new Cloud);
    compute (cloud, output, radius, inside, keep_organized);

    // Prepare output file name
    std::string filename = boost::filesystem::path(pcd_file).filename().string();
    
    // Save into the second file
    const std::string filepath = output_dir + '/' + filename;
    saveCloud (filepath, output);
  }
  return (0);
}


/* ---[ */
int
main (int argc, char** argv)
{
  pcl::console::print_info ("Filter a point cloud using the pcl::TfQuadraticXYZComparison. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool batch_mode = false;

  // Command line parsing
  float radius = default_radius;
  bool inside = default_inside;
  bool keep_organized = default_keep_organized;
  pcl::console::parse_argument (argc, argv, "-radius", radius);
  pcl::console::parse_argument (argc, argv, "-inside", inside);
  pcl::console::parse_argument (argc, argv, "-keep", keep_organized);
  std::string input_dir, output_dir;
  if (pcl::console::parse_argument (argc, argv, "-input_dir", input_dir) != -1)
  {
    PCL_INFO ("Input directory given as %s. Batch process mode on.\n", input_dir.c_str ());
    if (pcl::console::parse_argument (argc, argv, "-output_dir", output_dir) == -1)
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
    p_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () != 2)
    {
      pcl::console::print_error ("Need one input PCD file and one output PCD file to continue.\n");
      return (-1);
    }

    // Load the first file
    Cloud::Ptr cloud (new Cloud);
    if (!loadCloud (argv[p_file_indices[0]], cloud))
      return (-1);

    // Perform the feature estimation
    Cloud::Ptr output (new Cloud);
    compute (cloud, output, radius, inside, keep_organized);

    // Save into the second file
    saveCloud (argv[p_file_indices[1]], output);
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
      batchProcess (pcd_files, output_dir, radius, inside, keep_organized);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
}
