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
 * $Id$
 *
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/recognition/linemod.h>
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


std::vector<bool>
maskForegroundPoints (const PointCloudXYZRGBA::ConstPtr & input,
                      float min_depth, float max_depth, float max_height)
{
  std::vector<bool> foreground_mask (input->size (), false);
  
  // Mask off points outside the specified near and far depth thresholds
  pcl::IndicesPtr indices (new std::vector<int>);
  for (size_t i = 0; i < input->size (); ++i)
  {
    const float z = input->points[i].z;
    if (min_depth < z && z < max_depth)
    {
      foreground_mask[i] = true;
      indices->push_back (i);
    }
  }

  // Find the dominant plane between the specified near/far thresholds
  const float distance_threshold = 0.02;
  const int max_iterations = 500;
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold);
  seg.setMaxIterations (max_iterations);
  seg.setInputCloud (input);
  seg.setIndices (indices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  seg.segment (*inliers, *coefficients);  
  
  // Mask off the plane inliers
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    foreground_mask[inliers->indices[i]] = false;

  // Mask off any foreground points that are too high above the detected plane
  const std::vector<float> & c = coefficients->values;
  for (size_t i = 0; i < input->size (); ++i)
  {
    if (foreground_mask[i])
    {
      const pcl::PointXYZRGBA & p = input->points[i];
      float d = fabs (c[0]*p.x + c[1]*p.y + c[2]*p.z + c[3]);
      foreground_mask[i] = (d < max_height);
    }
  }

  // Just for temporary visualization
  PointCloudXYZRGBA visualization_cloud (*input);
  for (size_t i = 0; i < foreground_mask.size (); ++i)
  {
    if (!foreground_mask[i])
    {
      pcl::PointXYZRGBA & p = visualization_cloud.points[i];
      p.r = 64;
      p.g = 0;
      p.b = 0;
    }
    else
    {
      pcl::PointXYZRGBA & p = visualization_cloud.points[i];
      p.r = 0;
      p.g = 128;
      p.b = 0;
    }
  }
  pcl::io::savePCDFile ("temp.pcd", visualization_cloud);

  return (foreground_mask);
}

void
trainTemplate (const PointCloudXYZRGBA::ConstPtr & input, const std::vector<bool> &foreground_mask, 
               pcl::LINEMOD & linemod)
{
  
  pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
  color_grad_mod.setInputCloud (input);

  pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
  surface_norm_mod.setInputCloud (input);

  std::vector<pcl::QuantizableModality*> modalities (2);
  modalities[0] = &color_grad_mod;
  modalities[1] = &surface_norm_mod;

  size_t min_x (input->width), min_y (input->height), max_x (0), max_y (0);
  pcl::MaskMap mask_map (input->width, input->height);
  for (size_t j = 0; j < input->height; ++j)
  {
    for (size_t i = 0; i < input->width; ++i)
    {
      mask_map (i,j) = foreground_mask[j*input->width+i];
      if (foreground_mask[j*input->width+i])
      {
        min_x = std::min (min_x, i);
        max_x = std::max (max_x, i);
        min_y = std::min (min_y, j);
        max_y = std::max (max_y, j);
      }
    }
  }

  std::vector<pcl::MaskMap*> masks (2);
  masks[0] = &mask_map;
  masks[1] = &mask_map;

  pcl::RegionXY region;
  region.x = min_x;
  region.y = min_y;
  region.width = max_x - min_x + 1;
  region.height = max_y - min_y + 1;

  printf ("%d %d %d %d\n", region.x, region.y, region.width, region.height);

  linemod.createAndAddTemplate (modalities, masks, region);
}

void
compute (const PointCloudXYZRGBA::ConstPtr & input, float min_depth, float max_depth, float max_height,
         const char * template_filename)
{
  std::vector<bool> foreground_mask = maskForegroundPoints (input, min_depth, max_depth, max_height);

  pcl::LINEMOD linemod;
  trainTemplate (input, foreground_mask, linemod);

  linemod.saveTemplates (template_filename);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Train a linemod template. For more information, use: %s -h\n", argv[0]);

  if (argc < 6)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Load the input point cloud from the provided PCD file
  PointCloudXYZRGBA::Ptr cloud (new PointCloudXYZRGBA);
  if (!loadCloud (argv[1], *cloud)) 
    return (-1);

  // Train the LINE-MOD template and output it to the specified file
  compute (cloud, atof (argv[2]), atof (argv[3]), atof (argv[4]), argv[5]);

}

