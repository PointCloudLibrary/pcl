/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

//#include <png++/png.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

using PointCloudXYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input_cloud.pcd input_template.lmt\n", argv[0]);
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

std::vector<pcl::LINEMODDetection>
matchTemplates (const PointCloudXYZRGBA::ConstPtr & input, const pcl::LINEMOD & linemod)
{  
  pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
  color_grad_mod.setInputCloud (input);
  color_grad_mod.processInputData ();

  pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
  surface_norm_mod.setInputCloud (input);
  surface_norm_mod.processInputData ();

  std::vector<pcl::QuantizableModality*> modalities (2);
  modalities[0] = &color_grad_mod;
  modalities[1] = &surface_norm_mod;

  std::vector<pcl::LINEMODDetection> detections;
  linemod.matchTemplates (modalities, detections);

  return (detections);
}

void
compute (const PointCloudXYZRGBA::ConstPtr & input, const char * templates_filename)
{
  pcl::LINEMOD linemod;

  // Load the templates from disk
  linemod.loadTemplates (templates_filename);

  // Match the templates to the provided image
  std::vector<pcl::LINEMODDetection> detections = matchTemplates (input, linemod);

  // Output the position and score of the best match for each template
  for (std::size_t i = 0; i < detections.size (); ++i)
  {
    const LINEMODDetection & d = detections[i];
    printf ("%lu: %d %d %d %f\n", i, d.x, d.y, d.template_id, d.score);
  }

  /*// Visualization code for testing purposes (requires libpng++)
  int i = 0;
  png::image<png::rgb_pixel> image (640, 480);
  for (std::size_t y = 0; y < image.get_height (); ++y)
  {
    for (std::size_t x = 0; x < image.get_width (); ++x)
    {
      const pcl::PointXYZRGBA & p = (*input)[i++];
      image[y][x] = png::rgb_pixel(p.r, p.g, p.b);
    }
  }
  // Draw a green box around the object
  for (std::size_t i = 0; i < detections.size (); ++i)
  {
    const LINEMODDetection & d = detections[i];
    if (d.score < 0.6)
      continue;

    const pcl::SparseQuantizedMultiModTemplate & tmplt = linemod.getTemplate (d.template_id);
    int w = tmplt.region.width;
    int h = tmplt.region.height;
    for (int x = d.x; x < d.x+w; ++x)
    {
      image[d.y][x] = png::rgb_pixel(0, 255, 0);
      image[d.y+h-1][x] = png::rgb_pixel(0, 255, 0);
    }
    for (int y = d.y; y < d.y+h; ++y)
    {
      image[y][d.x] = png::rgb_pixel(0, 255, 0);
      image[y][d.x+w-1] = png::rgb_pixel(0, 255, 0);
    }
  }
  image.write("output.png");
  */
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Match a LINE-MOD template to an organized point cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Load the input point cloud from the provided PCD file
  PointCloudXYZRGBA::Ptr cloud (new PointCloudXYZRGBA);
  if (!loadCloud (argv[1], *cloud)) 
    return (-1);

  // Load the specified templates and match them to the provided input cloud
  compute (cloud, argv[2]);

}

