/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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
 */

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/generate.h>
#include <pcl/common/random.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::common;
using namespace pcl::console;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;
typedef const Cloud::ConstPtr ConstCloudPtr;

std::string default_distribution = "uniform";
float default_xmin = 0.0f;
float default_xmax = 1.0f;
float default_xmean = 0.0f;
float default_xstddev = 1.0f;
float default_ymin = 0.0f;
float default_ymax = 1.0f;
float default_ymean = 0.0f;
float default_ystddev = 1.0f;
float default_zmin = 0.0f;
float default_zmax = 1.0f;
float default_zmean = 0.0f;
float default_zstddev = 1.0f;
int default_size = 10000;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -distribution X = the distribution to be used (options: uniform / normal) (default: ");
  print_value ("%s", default_distribution.c_str ()); print_info (")\n");
  print_info ("                     -size X = number of points in cloud (default: ");
  print_value ("%d", default_size); print_info (")\n");
  print_info ("                Options for uniform distribution:\n");
  print_info ("                     -[x|y|z]min X = minimum for the [x|y|z] dimension (defaults: ");
  print_value ("%f, %f, %f", default_xmin, default_ymin, default_zmin); print_info (")\n");
  print_info ("                     -[x|y|z]max X = maximum for the [x|y|z] dimension (defaults: ");
  print_value ("%f, %f, %f", default_xmax, default_ymax, default_zmax); print_info (")\n");
  print_info ("                Options for normal distribution:\n");
  print_info ("                     -[x|y|z]mean X = mean for the [x|y|z] dimension (defaults: ");
  print_value ("%f, %f, %f", default_xmean, default_ymean, default_zmean); print_info (")\n");
  print_info ("                     -[x|y|z]stddev X = standard deviation for the [x|y|z] dimension (defaults: ");
  print_value ("%f, %f, %f", default_xstddev, default_ystddev, default_zstddev); print_info (")\n");
}

void
saveCloud (const std::string &filename, const Cloud &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  PCDWriter w;
  w.writeBinaryCompressed (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (find_switch (argc, argv, "-h"))
  {
    printHelp (argc, argv);
    return (0);
  }

  print_info ("Generate a random point cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Command line parsing
  std::string distribution = default_distribution;
  float xmin = default_xmin;
  float xmax = default_xmax;
  float xmean = default_xmean;
  float xstddev = default_xstddev;
  float ymin = default_ymin;
  float ymax = default_ymax;
  float ymean = default_ymean;
  float ystddev = default_ystddev;
  float zmin = default_zmin;
  float zmax = default_zmax;
  float zmean = default_zmean;
  float zstddev = default_zstddev;
  int size = default_size;
  parse_argument (argc, argv, "-distribution", distribution);
  parse_argument (argc, argv, "-xmin", xmin);
  parse_argument (argc, argv, "-xmax", xmax);
  parse_argument (argc, argv, "-xmean", xmean);
  parse_argument (argc, argv, "-xstddev", xstddev);
  parse_argument (argc, argv, "-ymin", ymin);
  parse_argument (argc, argv, "-ymax", ymax);
  parse_argument (argc, argv, "-ymean", ymean);
  parse_argument (argc, argv, "-ystddev", ystddev);
  parse_argument (argc, argv, "-zmin", zmin);
  parse_argument (argc, argv, "-zmax", zmax);
  parse_argument (argc, argv, "-zmean", zmean);
  parse_argument (argc, argv, "-zstddev", zstddev);
  parse_argument (argc, argv, "-size", size);

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 1)
  {
    print_error ("Need one output PCD file to continue.\n");
    return (-1);
  }

  // Perform the feature estimation
  Cloud output;

  // Estimate
  TicToc tt;
  tt.tic ();

  print_highlight (stderr, "Computing ");

  if (distribution == "uniform")
  {
    CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator;
    uint32_t seed = static_cast<uint32_t> (time (NULL));
    UniformGenerator<float>::Parameters x_params (xmin, xmax, seed++);
    generator.setParametersForX (x_params);
    UniformGenerator<float>::Parameters y_params (ymin, ymax, seed++);
    generator.setParametersForY (y_params);
    UniformGenerator<float>::Parameters z_params (zmin, zmax, seed++);
    generator.setParametersForZ (z_params);

    generator.fill (size, 1, output);
  }
  else if (distribution == "normal")
  {
    CloudGenerator<pcl::PointXYZ, NormalGenerator<float> > generator;
    uint32_t seed = static_cast<uint32_t> (time (NULL));
    NormalGenerator<float>::Parameters x_params (xmean, xstddev, seed++);
    generator.setParametersForX (x_params);
    NormalGenerator<float>::Parameters y_params (ymean, ystddev, seed++);
    generator.setParametersForY (y_params);
    NormalGenerator<float>::Parameters z_params (zmean, zstddev, seed++);
    generator.setParametersForZ (z_params);

    generator.fill (size, 1, output);
  }
  else
  {
    PCL_ERROR ("%s is not a valid generator! Quitting!\n", distribution.c_str ());
    return (0);
  }

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");

  // Save into the second file
  saveCloud (argv[p_file_indices[0]], output);
}

