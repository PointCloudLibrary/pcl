/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

\author Radu Bogdan Rusu

@b pcd_concatenate_points exemplifies how to concatenate the points of two PointClouds having the same fields.

**/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

  pcl::PointCloud<pcl::PointXYZ> cloud_all;
  for (size_t i = 0; i < file_indices.size (); ++i)
  {
    // Load the Point Cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile (argv[file_indices[i]], cloud);
    cloud_all += cloud;
  }

  pcl::PCDWriter writer;
  writer.writeBinaryCompressed ("output.pcd", cloud_all);
  
  return (0);
}
/* ]--- */
