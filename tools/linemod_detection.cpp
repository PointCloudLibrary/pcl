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



#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd  min_depth  max_depth  max_height  output_template.lmt\n", argv[0]);
  print_info ("  where options are:\n");
}

void printElapsedTimeAndNumberOfPoints (double t, int w, int h=1)
{
  print_info ("[done, "); print_value ("%g", t); print_info (" ms : "); 
  print_value ("%d", w*h); print_info (" points]\n");
}

bool
loadCloud (const std::string & filename, PointCloudXYZRGBA & cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);

  printElapsedTimeAndNumberOfPoints (tt.toc (), cloud.width, cloud.height);

  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}



/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Train one or more linemod templates. For more information, use: %s -h\n", argv[0]);

  // If no arguments are given, print the help text
  if (argc == 1)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the gradient magnitude threshold
  float grad_mag_thresh = 10.0f;
  parse_argument (argc, argv, "-grad_mag_thresh", grad_mag_thresh);

  // Parse the detection threshold
  float detect_thresh = 0.75f;
  parse_argument (argc, argv, "-detect_thresh", detect_thresh);

  // Parse the command line arguments for .lmt files
  std::vector<int> lmt_file_indices;
  lmt_file_indices = parse_file_extension_argument (argc, argv, ".lmt");
  if (lmt_file_indices.empty ())
  {
    print_error ("Need at least one input LMT file.\n");
    return (-1);
  }

  LineRGBD<PointXYZRGBA> line_rgbd;
  line_rgbd.setGradientMagnitudeThreshold (grad_mag_thresh);
  line_rgbd.setDetectionThreshold (detect_thresh);

  // Load the template LMT and PCD files
  for (size_t i = 0; i < lmt_file_indices.size (); ++i)
  {
    // Load the LMT file
    std::string lmt_filename = argv[lmt_file_indices[i]];
    line_rgbd.loadTemplates (lmt_filename);
  }

  // Load the input PCD file
  std::string input_filename;
  if (parse_argument (argc, argv, "-input", input_filename) < 0)
    return (-1);
  PointCloudXYZRGBA::Ptr cloud (new PointCloudXYZRGBA);
  if (!loadCloud (input_filename, *cloud)) 
    return (-1);

  // Detect objects
  line_rgbd.setInputCloud (cloud);
  line_rgbd.setInputColors (cloud);

  std::vector<LineRGBD<PointXYZRGBA>::Detection> detections;
  line_rgbd.detect (detections);

  for (size_t i = 0; i < detections.size (); ++i)
  {
    const LineRGBD<PointXYZRGBA>::Detection & d = detections[i];
    const BoundingBoxXYZ & bb = d.bounding_box;
    print_info ("%lu %lu %f (%f %f %f) (%f %f %f)\n", 
                d.detection_id, d.template_id, d.response,
                bb.x, bb.y, bb.z, bb.width, bb.height, bb.depth);
  }
}
