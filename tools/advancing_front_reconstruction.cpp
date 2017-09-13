/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2017-, Southwest Research Institute
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/advancing_front.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.vtk <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X         = radius used for local surface reconstruction at each point. (Required)\n");
  print_info ("                     -order X          = the polynomial order for local surface reconstruction at each point. (default: ");
  print_value ("%d", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_POLYNOMIAL_ORDER); print_info (")\n");
  print_info ("                     -rho X            = used to control mesh triangle size (0 > rho > 1.570796). (default: ");
  print_value ("%f", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_RHO); print_info (")\n");
  print_info ("                     -reduction X      = defines how fast the mesh triangles can grow and shrink (0 > reduction < 1). (default: ");
  print_value ("%f", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_REDUCTION); print_info (")\n");
  print_info ("                     -boundary_angle X = threshold used to determine if a point is on the boundary of the point cloud. (default: ");
  print_value ("%f", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_BOUNDARY_ANGLE_THRESHOLD); print_info (")\n");
  print_info ("                     -sample_size X    = the number of sample triangles to generate. (default: ");
  print_value ("%d", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_SAMPLE_SIZE); print_info (")\n");
#ifdef _OPENMP
  print_info ("                     -threads X        = the number of threads to use. (default: ");
  print_value ("%d", AfrontMesher<PointXYZ>::AFRONT_DEFAULT_THREADS); print_info (")\n");
#endif
}

bool
loadCloud (const std::string &filename, const std::string &extension, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (extension == ".pcd")
  {
    if (loadPCDFile (filename, cloud) < 0)
      return (false);
  }
  else if (extension == ".ply")
  {
    if (loadPLYFile (filename, cloud) < 0)
      return (false);
  }
  else
  {
    return (false);
  }

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const std::string &extension, const PolygonMesh &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  if (extension == ".vtk")
    saveVTKFile (filename, output);
  else if (extension == ".ply")
    savePLYFile (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Compute the surface reconstruction of a point cloud using the advancing front algorithm. For more information, use: %s -h\n", argv[0]);

  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse command line arguments
  std::string input_file = argv[1];
  std::string input_extension = boost::filesystem::extension(input_file);
  if (input_extension != ".pcd" && input_extension != ".ply")
  {
    print_error("Only input file types supported are: pcd and ply.\n");
    return (-1);
  }

  std::string output_file = argv[2];
  std::string output_extension = boost::filesystem::extension(output_file);
  if (output_extension != ".vtk" && output_extension != ".ply")
  {
    print_error("Only output file types supported are: vtk and ply.\n");
    return (-1);
  }

  // Parse command line arguments
  double radius = 0;
  parse_argument (argc, argv, "-radius", radius);
  if (radius <= 0)
  {
    print_error("Argument -radius is required and must be greater than zero.\n");
    return (-1);
  }
  print_info ("Setting a radius of: "); print_value ("%f\n", radius);

  int order = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_POLYNOMIAL_ORDER;
  parse_argument (argc, argv, "-order", order);
  print_info ("Setting a polynomial order of: "); print_value ("%d\n", order);

  double rho = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_RHO;
  parse_argument (argc, argv, "-rho", rho);
  print_info ("Setting a rho of: "); print_value ("%f\n", rho);

  double reduction = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_REDUCTION;
  parse_argument (argc, argv, "-reduction", reduction);
  print_info ("Setting a reduction of: "); print_value ("%f\n", reduction);

  double boundary_angle = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_BOUNDARY_ANGLE_THRESHOLD;
  parse_argument (argc, argv, "-boundary_angle", boundary_angle);
  print_info ("Setting a boundary angle threshold of: "); print_value ("%f\n", boundary_angle);

  int sample_size = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_SAMPLE_SIZE;
  parse_argument (argc, argv, "-sample_size", sample_size);
  print_info ("Setting a sample size of: "); print_value ("%d\n", sample_size);

#ifdef _OPENMP
  int threads = AfrontMesher<PointXYZ>::AFRONT_DEFAULT_THREADS;
  parse_argument (argc, argv, "-threads", threads);
  print_info ("Setting a number of threads of: "); print_value ("%d\n", threads);
#endif

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (input_file, input_extension, *cloud))
    return (-1);

  PointCloud<PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<PointXYZ> ());
  fromPCLPointCloud2 (*cloud,  *xyz_cloud);

  // Apply the marching cubes algorithm
  PolygonMesh output;
  AfrontMesher<PointXYZ> mesher;
  mesher.setRho (rho);
  mesher.setReduction (reduction);
  mesher.setSearchRadius (radius);
  mesher.setPolynomialOrder (order);
  mesher.setBoundaryAngleThreshold (boundary_angle);
  mesher.setInputCloud (xyz_cloud);
  mesher.setSampleSize (sample_size);
#ifdef _OPENMP
  mesher.setNumberOfThreads (threads);
#endif

  TicToc tt;
  tt.tic ();

  print_highlight ("Computing ");
  mesher.reconstruct (output);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");

  // Save into the second file
  saveCloud (output_file, output_extension, output);
}
