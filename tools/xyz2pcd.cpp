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

#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <boost/algorithm/string/split.hpp> // for split

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.xyz output.pcd\n", argv[0]);
}

bool
loadCloud (const std::string &filename, PointCloud<PointXYZ> &cloud)
{
  std::ifstream fs;
  fs.open (filename.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno)); 
    fs.close ();
    return (false);
  }
  
  std::string line;
  std::vector<std::string> st;

  while (!fs.eof ())
  {
    std::getline (fs, line);
    // Ignore empty lines
    if (line.empty())
      continue;

    // Tokenize the line
    boost::trim (line);
    boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

    if (st.size () != 3)
      continue;

    cloud.push_back (PointXYZ (float (atof (st[0].c_str ())), float (atof (st[1].c_str ())), float (atof (st[2].c_str ()))));
  }
  fs.close ();

  cloud.width = cloud.size (); cloud.height = 1; cloud.is_dense = true;
  return (true);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a simple XYZ file to PCD format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .ply files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> xyz_file_indices = parse_file_extension_argument (argc, argv, ".xyz");
  if (pcd_file_indices.size () != 1 || xyz_file_indices.size () != 1)
  {
    print_error ("Need one input XYZ file and one output PCD file.\n");
    return (-1);
  }

  // Load the first file
  PointCloud<PointXYZ> cloud;
  if (!loadCloud (argv[xyz_file_indices[0]], cloud)) 
    return (-1);

  // Convert to PCD and save
  PCDWriter w;
  w.writeBinaryCompressed (argv[pcd_file_indices[0]], cloud);
}

