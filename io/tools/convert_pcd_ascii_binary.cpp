/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: convert_pcd_ascii_binary.cpp 33162 2010-03-10 07:41:56Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b convert_pcd_ascii_binary converts PCD (Point Cloud Data) files from ascii to binary and viceversa.

 **/

#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <string>

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr << "Syntax is: " << argv[0] << " <file_in.pcd> <file_out.pcd> 0/1/2 (ascii/binary/binary_compressed) [precision (ASCII)]" << std::endl;
    return (-1);
  }

  pcl::PCLPointCloud2 cloud;
  Eigen::Vector4f origin; Eigen::Quaternionf orientation;

  if (pcl::io::loadPCDFile (std::string (argv[1]), cloud, origin, orientation) < 0)
  {
    std::cerr << "Unable to load " << argv[1] << std::endl;
    return (-1);
  }

  int type = atoi (argv[3]);

  std::cerr << "Loaded a point cloud with " << cloud.width * cloud.height << 
               " points (total size is " << cloud.data.size () << 
               ") and the following channels: " << pcl::getFieldsList (cloud) << std::endl;

  pcl::PCDWriter w;
  if (type == 0)
  {
    std::cerr << "Saving file " << argv[2] << " as ASCII." << std::endl;
    w.writeASCII (std::string (argv[2]), cloud, origin, orientation, (argc == 5) ? atoi (argv[4]) : 7);
  }
  else if (type == 1)
  {
    std::cerr << "Saving file " << argv[2] << " as binary." << std::endl;
    w.writeBinary (std::string (argv[2]), cloud, origin, orientation);
  }
  else if (type == 2)
  {
    std::cerr << "Saving file " << argv[2] << " as binary_compressed." << std::endl;
    w.writeBinaryCompressed (std::string (argv[2]), cloud, origin, orientation);
  }
}
/* ]--- */
