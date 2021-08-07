/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_iso_level = 0.0f;
int default_hoppe_or_rbf = 0;
float default_extend_percentage = 0.0f;
int default_grid_res = 50;
float default_off_surface_displacement = 0.01f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.vtk <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -grid_res X     = the resolution of the grid (cubic grid) (default: ");
  print_value ("%d", default_grid_res); print_info (")\n");
  print_info ("                     -iso_level X    = the iso level of the surface to be extracted (default: ");
  print_value ("%f", default_iso_level); print_info (")\n");
  print_info ("                     -hoppe X        = use the Hoppe signed distance function (MarchingCubesHoppe)\n");
  print_info ("                     -rbf X          = use the RBF signed distance function (MarchingCubesRBF)\n");
  print_info ("                     -extend X       = the percentage of the bounding box to extend the grid by (default: ");
  print_value ("%f", default_extend_percentage); print_info (")\n");
  print_info ("                     -displacement X = the displacement value for the off-surface points (only for RBF) (default: ");
  print_value ("%f", default_off_surface_displacement); print_info (")\n");
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
         int hoppe_or_rbf, float iso_level, int grid_res, float extend_percentage, float off_surface_displacement)
{
  PointCloud<PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<PointNormal> ());
  fromPCLPointCloud2 (*input, *xyz_cloud);

  MarchingCubes<PointNormal> *mc;
  if (hoppe_or_rbf == 0)
    mc = new MarchingCubesHoppe<PointNormal> ();
  else
  {
    mc = new MarchingCubesRBF<PointNormal> ();
    (reinterpret_cast<MarchingCubesRBF<PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
  }

  mc->setIsoLevel (iso_level);
  mc->setGridResolution (grid_res, grid_res, grid_res);
  mc->setPercentageExtendGrid (extend_percentage);
  mc->setInputCloud (xyz_cloud);

  TicToc tt;
  tt.tic ();

  print_highlight ("Computing ");
  mc->reconstruct (output);
  delete mc;

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
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
  print_info ("Compute the surface reconstruction of a point cloud using the marching cubes algorithm (pcl::surface::MarchingCubesHoppe or pcl::surface::MarchingCubesRBF. For more information, use: %s -h\n", argv[0]);

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
  int hoppe_or_rbf = default_hoppe_or_rbf;
  bool ok = false;
  parse_argument (argc, argv, "-hoppe", ok);
  if (ok)
  {
    hoppe_or_rbf = 0;
    print_info ("Selected algorithm: MarchingCubesHoppe\n");
  }
  ok = false;
  parse_argument (argc, argv, "-rbf", ok);
  if (ok)
  {
    hoppe_or_rbf = 1;
    print_info ("Selected algorithm: MarchingCubesRBF\n");
  }

  float iso_level = default_iso_level;
  parse_argument (argc, argv, "-iso_level", iso_level);
  print_info ("Setting an iso level of: "); print_value ("%f\n", iso_level);

  int grid_res = default_grid_res;
  parse_argument (argc, argv, "-grid_res", grid_res);
  print_info ("Setting a cubic grid resolution of: "); print_value ("%d\n", grid_res);

  float extend_percentage = default_extend_percentage;
  parse_argument (argc, argv, "-extend", extend_percentage);
  print_info ("Setting an extend percentage of: "); print_value ("%f\n", extend_percentage);

  float off_surface_displacement = default_off_surface_displacement;
  parse_argument (argc, argv, "-displacement", off_surface_displacement);
  print_info ("Setting an off-surface displacement of: "); print_value ("%f\n", off_surface_displacement);

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[pcd_file_indices[0]], *cloud))
    return (-1);

  // Apply the marching cubes algorithm
  PolygonMesh output;
  compute (cloud, output, hoppe_or_rbf, iso_level, grid_res, extend_percentage, off_surface_displacement);

  // Save into the second file
  saveCloud (argv[vtk_file_indices[0]], output);
}

