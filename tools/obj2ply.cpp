/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
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

#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

void
printHelp (int,
           char **argv)
{
  PCL_ERROR ("Syntax is: %s [-format 0|1] [-use_camera 0|1] input.obj output.ply\n", argv[0]);
  PCL_ERROR ("format = 0 writes a binary file (default behavior), format = 1 writes an ASCII file.\n");
}

int
main (int argc,
      char** argv)
{
  PCL_INFO ("Convert a OBJ file to PLY format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for files
  std::vector<int> obj_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".obj");
  std::vector<int> ply_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
  if (obj_file_indices.size () != 1 || ply_file_indices.size () != 1)
  {
    PCL_ERROR ("Need one input OBJ file and one output PLY file.\n");
    return (-1);
  }

  // Command line parsing
  bool ascii = false;
  pcl::console::parse_argument (argc, argv, "-format", ascii);
  PCL_INFO ("PLY output format: %s\n", ascii ? "ascii" : "binary");

  // Load the first file
  pcl::PolygonMesh mesh;
  if (pcl::io::loadOBJFile (argv[obj_file_indices[0]], mesh) != 0)
  {
    PCL_ERROR ("Error loading OBJ file\n");
    return (-1);
  }

  if (ascii)
  {
    if (pcl::io::savePLYFile (argv[ply_file_indices[0]], mesh) != 0)
    {
      PCL_ERROR ("Error writing PLY file\n");
      return (-1);
    }
  }
  else if (pcl::io::savePLYFileBinary (argv[ply_file_indices[0]], mesh) != 0)
  {
    PCL_ERROR ("Error writing PLY file\n");
    return (-1);
  }

  return (0);
}
