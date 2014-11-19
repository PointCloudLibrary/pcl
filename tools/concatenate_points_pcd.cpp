/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/**

@b pcd_concatenate_points exemplifies how to concatenate the points of two PointClouds having the same fields.

**/

#include <iostream>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse command line arguments for file names. 
  * Returns: a vector with file names indices.
  * \param argc 
  * \param argv
  * \param extension
  */
std::vector<int>
parseFileExtensionArgument (int argc, char** argv, std::string extension)
{
  std::vector<int> indices;
  for (int i = 1; i < argc; ++i)
  {
    std::string fname = std::string (argv[i]);
    
    // Needs to be at least 4: .ext
    if (fname.size () <= 4)
      continue;
    
    // For being case insensitive
    std::transform (fname.begin (), fname.end (), fname.begin (), tolower);
    std::transform (extension.begin (), extension.end (), extension.begin (), tolower);
    
    // Check if found
    std::string::size_type it;
    if ((it = fname.find (extension)) != std::string::npos)
    {
      // Additional check: we want to be able to differentiate between .p and .png
      if ((extension.size () - (fname.size () - it)) == 0)
        indices.push_back (i);
    }
  }
  return (indices);
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  using namespace pcl::console;
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (pcl::io::loadPCDFile (filename, cloud, translation, orientation) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  using namespace pcl::console;
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  pcl::PCDWriter w;
  w.writeBinaryCompressed (filename, output, translation, orientation);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}


/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Syntax is: " << argv[0] << " <filename 1..N.pcd>" << std::endl;
    std::cerr << "Result will be saved to output.pcd" << std::endl;
    return (-1);
  }

  std::vector<int> file_indices = parseFileExtensionArgument (argc, argv, ".pcd");

  //pcl::PointCloud<pcl::PointXYZ> cloud_all;
  pcl::PCLPointCloud2 cloud_all;
  for (size_t i = 0; i < file_indices.size (); ++i)
  {
    // Load the Point Cloud
    pcl::PCLPointCloud2 cloud;
    loadCloud (argv[file_indices[i]], cloud);
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //pcl::io::loadPCDFile (argv[file_indices[i]], cloud);
    //cloud_all += cloud;
    pcl::concatenatePointCloud (cloud_all, cloud, cloud_all);
    PCL_INFO ("Total number of points so far: %u. Total data size: %lu bytes.\n", cloud_all.width * cloud_all.height, cloud_all.data.size ());
  }

  saveCloud ("output.pcd", cloud_all);
  
  return (0);
}
/* ]--- */
