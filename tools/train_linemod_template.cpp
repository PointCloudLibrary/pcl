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

#include <cmath>

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

using PointCloudXYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input1.pcd input2.pcd input3.pcd (etc.)\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("           -min_depth z_min   = the depth of the near clipping plane\n");
  print_info ("           -max_depth z_max   = the depth of the far clipping plane\n");
  print_info ("           -max_height y_max  = the height of the vertical clipping plane\n");
  print_info ("Two new template files will be created for each input file.  They will append ");
  print_info ("the following suffixes to the original filename:\n");
  print_info ("   _template.pcd (A PCD containing segmented points)\n");
  print_info ("   _template.sqmmt (A file storing LINEMOD's 'Sparse Quantized Multi-Modal Template' representation)\n");

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
  pcl::IndicesPtr indices (new pcl::Indices);
  for (std::size_t i = 0; i < input->size (); ++i)
  {
    const float z = (*input)[i].z;
    if (min_depth < z && z < max_depth)
    {
      foreground_mask[i] = true;
      indices->push_back (static_cast<int> (i));
    }
  }

  // Find the dominant plane between the specified near/far thresholds
  const float distance_threshold = 0.02f;
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
  for (const auto &index : inliers->indices)
    foreground_mask[index] = false;

  // Mask off any foreground points that are too high above the detected plane
  const std::vector<float> & c = coefficients->values;
  for (std::size_t i = 0; i < input->size (); ++i)
  {
    if (foreground_mask[i])
    {
      const pcl::PointXYZRGBA & p = (*input)[i];
      float d = std::abs (c[0]*p.x + c[1]*p.y + c[2]*p.z + c[3]);
      foreground_mask[i] = (d < max_height);
    }
  }

  return (foreground_mask);
}

void
trainTemplate (const PointCloudXYZRGBA::ConstPtr & input, const std::vector<bool> &foreground_mask,
               pcl::LINEMOD & linemod)
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

  std::size_t min_x (input->width), min_y (input->height), max_x (0), max_y (0);
  pcl::MaskMap mask_map (input->width, input->height);
  for (std::size_t j = 0; j < input->height; ++j)
  {
    for (std::size_t i = 0; i < input->width; ++i)
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
  region.x = static_cast<int> (min_x);
  region.y = static_cast<int> (min_y);
  region.width = static_cast<int> (max_x - min_x + 1);
  region.height = static_cast<int> (max_y - min_y + 1);

  printf ("%d %d %d %d\n", region.x, region.y, region.width, region.height);

  linemod.createAndAddTemplate (modalities, masks, region);
}

void
compute (const PointCloudXYZRGBA::ConstPtr & input, float min_depth, float max_depth, float max_height,
         const std::string & template_pcd_filename, const std::string & template_sqmmt_filename)
{
  // Segment the foreground object
  std::vector<bool> foreground_mask = maskForegroundPoints (input, min_depth, max_depth, max_height);

  // Save the masked template cloud (masking with NaNs to preserve its organized structure)
  PointCloudXYZRGBA template_cloud (*input);
  for (std::size_t i = 0; i < foreground_mask.size (); ++i)
  {
    if (!foreground_mask[i])
    {
      pcl::PointXYZRGBA & p = template_cloud[i];
      p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
  pcl::io::savePCDFile (template_pcd_filename, template_cloud);

  // Create a LINEMOD template
  pcl::LINEMOD linemod;
  trainTemplate (input, foreground_mask, linemod);

  // Save the LINEMOD template
  linemod.saveTemplates (template_sqmmt_filename.c_str());
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

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.empty ())
  {
    print_error ("Need at least one input PCD file.\n");
    return (-1);
  }

  // Parse the min_depth, max_depth, and max_height parameters
  float min_depth = 0;
  parse_argument (argc, argv, "-min_depth", min_depth);

  float max_depth = std::numeric_limits<float>::max ();
  parse_argument (argc, argv, "-max_depth", max_depth);

  float max_height = std::numeric_limits<float>::max ();
  parse_argument (argc, argv, "-max_height", max_height);

  int error_code = 0;
  bool processed_at_least_one_pcd = false;

  // Segment and create templates for each input file
  for (const int &p_file_index : p_file_indices)
  {
    // Load input file
    const std::string input_filename = argv[p_file_index];
    PointCloudXYZRGBA::Ptr cloud (new PointCloudXYZRGBA);

    if (!loadCloud (input_filename, *cloud))
    {
      error_code = -1;
      std::string warn_msg = "Could not load point cloud from file: " + input_filename + "\n";
      print_warn (warn_msg.c_str ());
      continue;
    }

    if (!cloud->isOrganized())
    {
      std::string warn_msg = "Unorganized point cloud detected. Skipping file " + input_filename + "\n";
      print_warn(warn_msg.c_str());
      continue;
    }
    else
    {
      processed_at_least_one_pcd = true;
    }

    // Construct output filenames
    std::string sqmmt_filename = input_filename;
    sqmmt_filename.replace(sqmmt_filename.length () - 4, 13, "_template.sqmmt");

    std::string pcd_filename = input_filename;
    pcd_filename.replace(pcd_filename.length () - 4, 13, "_template.pcd");

    std::cout << sqmmt_filename << std::endl;
    std::cout << pcd_filename << std::endl;

    // Train the LINE-MOD template and output it to the specified file
    compute (cloud, min_depth, max_depth, max_height, pcd_filename, sqmmt_filename);
  }

  if (!processed_at_least_one_pcd)
  {
    print_error("All input pcd files are unorganized.\n");
  }

  return error_code;
}
