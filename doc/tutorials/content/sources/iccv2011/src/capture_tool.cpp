/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#include "openni_capture.h"

#include <string>
#include <pcl/console/parse.h>

int 
main (int argc, char ** argv)
{
  if (argc < 2) 
  {
    pcl::console::print_info ("Syntax is: %s output_filename <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -n nr_frames ... Number of frames to capture (default = 1)\n");
    pcl::console::print_info ("  Note: The output filename should be provided without the .pcd extension.\n");
    pcl::console::print_info ("        File names will have frame numbers and extension appended to them.\n");
    pcl::console::print_info ("        (e.g., 'output' will become 'output_1.pcd', 'output_2.pcd', etc.)\n");
    return (1);
  }

  // Determine the number of frames to capture
  int nr_frames = 1; // Default value
  pcl::console::parse_argument (argc, argv, "-n", nr_frames);

  OpenNICapture camera;
  camera.setTriggerMode (true);
  for (int i = 0; i < nr_frames; ++i)
  {
    std::string foo = argv[1];
    std::stringstream filename (foo);
    filename << argv[1] << "_" << i << ".pcd";
    PCL_INFO ("%s\n", filename.str().c_str());
    camera.snapAndSave (filename.str ());
  }

  return (0);
}
