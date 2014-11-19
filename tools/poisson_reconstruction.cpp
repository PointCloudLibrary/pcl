/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/poisson.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int default_depth = 8;
int default_solver_divide = 8;
int default_iso_divide = 8;
float default_point_weight = 4.0f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.vtk <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -depth X          = set the maximum depth of the tree that will be used for surface reconstruction (default: ");
  print_value ("%d", default_depth); print_info (")\n");
  print_info ("                     -solver_divide X  = set the the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation (default: ");
  print_value ("%d", default_solver_divide); print_info (")\n");
  print_info ("                     -iso_divide X     = Set the depth at which a block iso-surface extractor should be used to extract the iso-surface (default: ");
  print_value ("%d", default_iso_divide); print_info (")\n");
  print_info ("                     -point_weight X   = Specifies the importance that interpolation of the point samples is given in the formulation of the screened Poisson equation. The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0. (default: ");
  print_value ("%f", default_point_weight); print_info (")\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
         int depth, int solver_divide, int iso_divide, float point_weight)
{
  PointCloud<PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<PointNormal> ());
  fromPCLPointCloud2 (*input, *xyz_cloud);

  print_info ("Using parameters: depth %d, solverDivide %d, isoDivide %d\n", depth, solver_divide, iso_divide);

	Poisson<PointNormal> poisson;
	poisson.setDepth (depth);
	poisson.setSolverDivide (solver_divide);
	poisson.setIsoDivide (iso_divide);
  poisson.setPointWeight (point_weight);
  poisson.setInputCloud (xyz_cloud);

  TicToc tt;
  tt.tic ();
  print_highlight ("Computing ...");
  poisson.reconstruct (output);

  print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

void
saveCloud (const std::string &filename, const PolygonMesh &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  saveVTKFile (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Compute the surface reconstruction of a point cloud using the Poisson surface reconstruction (pcl::surface::Poisson). For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> pcd_file_indices;
  pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
  {
    print_error ("Need one input PCD file and one output VTK file to continue.\n");
    return (-1);
  }

  std::vector<int> vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  if (vtk_file_indices.size () != 1)
  {
    print_error ("Need one output VTK file to continue.\n");
    return (-1);
  }

  // Command line parsing
  int depth = default_depth;
  parse_argument (argc, argv, "-depth", depth);
  print_info ("Using a depth of: "); print_value ("%d\n", depth);

  int solver_divide = default_solver_divide;
  parse_argument (argc, argv, "-solver_divide", solver_divide);
  print_info ("Setting solver_divide to: "); print_value ("%d\n", solver_divide);

  int iso_divide = default_iso_divide;
  parse_argument (argc, argv, "-iso_divide", iso_divide);
  print_info ("Setting iso_divide to: "); print_value ("%d\n", iso_divide);

  float point_weight = default_point_weight;
  parse_argument (argc, argv, "-point_weight", point_weight);
  print_info ("Setting point_weight to: "); print_value ("%f\n", point_weight);

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[pcd_file_indices[0]], *cloud))
    return (-1);

  // Apply the Poisson surface reconstruction algorithm
  PolygonMesh output;
  compute (cloud, output, depth, solver_divide, iso_divide, point_weight);

  // Save into the second file
  saveCloud (argv[vtk_file_indices[0]], output);
}

